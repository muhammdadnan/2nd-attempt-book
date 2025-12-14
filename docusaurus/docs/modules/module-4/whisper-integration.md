# Whisper Integration

In this chapter, we'll explore how to integrate OpenAI's Whisper ASR (Automatic Speech Recognition) model into our humanoid robot's VLA pipeline. Whisper provides state-of-the-art speech-to-text capabilities that enable natural human-robot interaction through voice commands.

## Introduction to Whisper for Robotics

### Why Whisper for Robotics?

Whisper offers several advantages for robotic applications:
- **Robustness**: Handles various accents, background noise, and speaking styles
- **Multilingual**: Supports multiple languages for international applications
- **Open Source**: Free to use and modify for robotics applications
- **Real-time capable**: With proper optimization, can work in real-time
- **Context aware**: Can be fine-tuned for specific robotic vocabularies

### Whisper Architecture Overview

```
Audio Input → Feature Extraction → Encoder → Decoder → Text Output
              (Mel Spectrogram)   (Transformer) (Transformer) (Tokens)
```

## Whisper Installation and Setup

### Prerequisites

First, install Whisper and related dependencies:

```bash
# Install Whisper and related packages
pip install openai-whisper
pip install torch torchvision torchaudio
pip install pyaudio sounddevice
pip install rclpy std_msgs sensor_msgs
```

### Optimized Whisper Installation

For GPU acceleration and better performance:

```bash
# Install with GPU support
pip install openai-whisper[cuda]

# Alternative: Install from source for latest features
git clone https://github.com/openai/whisper.git
cd whisper
pip install -e .
```

## ROS 2 Whisper Node Implementation

### Basic Whisper ROS Node

```python
# whisper_ros_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image
import whisper
import torch
import numpy as np
import io
from scipy.io import wavfile
import tempfile
import os

class WhisperROSNode(Node):
    def __init__(self):
        super().__init__('whisper_asr_node')

        # Initialize Whisper model
        self.model = None
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Load model based on performance requirements
        self.model_size = self.declare_parameter('model_size', 'small').value
        self._load_whisper_model()

        # Audio input parameters
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_duration = 0.5  # Process audio in 0.5 second chunks
        self.vad_threshold = 0.3  # Voice activity detection threshold

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )

        self.transcription_pub = self.create_publisher(
            String,
            '/whisper/transcription',
            10
        )

        self.is_listening_pub = self.create_publisher(
            Bool,
            '/whisper/is_listening',
            10
        )

        # Internal state
        self.audio_buffer = np.array([])
        self.is_processing = False
        self.min_audio_length = self.sample_rate * 1  # Minimum 1 second for transcription

        # Performance monitoring
        self.processing_times = []
        self.total_audio_processed = 0

        self.get_logger().info(f'Whisper ASR node initialized on {self.device}')
        self.get_logger().info(f'Using {self.model_size} model')

    def _load_whisper_model(self):
        """Load Whisper model with appropriate settings"""
        try:
            self.get_logger().info(f'Loading Whisper {self.model_size} model...')

            # Load model with specified size
            self.model = whisper.load_model(self.model_size).to(self.device)

            # Enable fp16 for faster inference on GPU
            if self.device == "cuda":
                self.model = self.model.half()

            self.get_logger().info('Whisper model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            # Fall back to tiny model if loading fails
            self.model_size = 'tiny'
            self.model = whisper.load_model(self.model_size).to(self.device)
            self.get_logger().warn(f'Loaded fallback model: {self.model_size}')

    def audio_callback(self, msg):
        """Process incoming audio data"""
        if self.is_processing:
            # Skip if currently processing to avoid overload
            return

        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Resample if needed (Whisper expects 16kHz)
            if msg.rate != 16000:
                # Simple resampling - in practice, use proper resampling
                audio_array = self._resample_audio(audio_array, msg.rate, 16000)

            # Append to buffer
            self.audio_buffer = np.concatenate([self.audio_buffer, audio_array])

            # Check if we have enough audio to process
            if len(self.audio_buffer) >= self.min_audio_length:
                self.is_processing = True

                # Publish listening status
                listening_msg = Bool()
                listening_msg.data = True
                self.is_listening_pub.publish(listening_msg)

                # Process in separate thread to avoid blocking
                self.process_audio_thread = threading.Thread(target=self._process_audio_buffer)
                self.process_audio_thread.start()

        except Exception as e:
            self.get_logger().error(f'Audio processing error: {e}')

    def _process_audio_buffer(self):
        """Process accumulated audio buffer in separate thread"""
        try:
            # Extract a reasonable chunk for processing
            chunk_size = min(len(self.audio_buffer), self.sample_rate * 10)  # Max 10 seconds
            audio_chunk = self.audio_buffer[:chunk_size]

            # Clear processed portion from buffer
            self.audio_buffer = self.audio_buffer[chunk_size:]

            # Perform transcription
            start_time = self.get_clock().now()
            result = self._transcribe_audio(audio_chunk)
            end_time = self.get_clock().now()

            processing_time = (end_time.nanoseconds - start_time.nanoseconds) / 1e9
            self.processing_times.append(processing_time)

            # Publish result if successful
            if result and result.text.strip():
                transcription_msg = String()
                transcription_msg.data = result.text.strip()
                self.transcription_pub.publish(transcription_msg)

                self.get_logger().info(f'Transcribed: "{result.text.strip()}" (took {processing_time:.3f}s)')

            # Update performance metrics
            self.total_audio_processed += len(audio_chunk) / self.sample_rate

        except Exception as e:
            self.get_logger().error(f'Audio transcription error: {e}')
        finally:
            self.is_processing = False

            # Publish listening status
            listening_msg = Bool()
            listening_msg.data = False
            self.is_listening_pub.publish(listening_msg)

    def _transcribe_audio(self, audio_array):
        """Transcribe audio using Whisper model"""
        try:
            # Ensure audio is in correct format
            if len(audio_array.shape) > 1:
                audio_array = audio_array.mean(axis=1)  # Convert to mono if stereo

            # Pad or trim audio to reasonable length (max 30 seconds for Whisper)
            if len(audio_array) > self.sample_rate * 30:
                audio_array = audio_array[:self.sample_rate * 30]

            # Transcribe using Whisper
            result = self.model.transcribe(
                audio_array,
                language='en',  # Set to your robot's primary language
                temperature=0.0,  # Deterministic output
                compression_ratio_threshold=2.4,
                logprob_threshold=-1.0,
                no_speech_threshold=0.6
            )

            return result

        except Exception as e:
            self.get_logger().error(f'Whisper transcription error: {e}')
            return None

    def _resample_audio(self, audio, original_sr, target_sr):
        """Simple audio resampling (use librosa or scipy for better quality)"""
        if original_sr == target_sr:
            return audio

        # Simple resampling using numpy (not ideal for production)
        duration = len(audio) / original_sr
        target_length = int(duration * target_sr)

        # Use numpy's interp for simple resampling
        original_times = np.linspace(0, len(audio)-1, len(audio))
        target_times = np.linspace(0, len(audio)-1, target_length)
        resampled = np.interp(target_times, original_times, audio)

        return resampled

    def get_performance_metrics(self):
        """Get performance metrics"""
        if not self.processing_times:
            return {'avg_processing_time': 0.0, 'total_processed': 0.0}

        avg_time = sum(self.processing_times[-50:]) / len(self.processing_times[-50:])  # Last 50 samples
        return {
            'avg_processing_time': avg_time,
            'total_audio_processed': self.total_audio_processed,
            'recent_transcriptions': len(self.processing_times[-50:])
        }

def main(args=None):
    rclpy.init(args=args)
    node = WhisperASRNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Whisper ASR node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Whisper Configuration

### Optimized Whisper Node with Streaming

```python
# streaming_whisper_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image
import whisper
import torch
import numpy as np
from collections import deque
import threading
import time

class StreamingWhisperNode(Node):
    def __init__(self):
        super().__init__('streaming_whisper_node')

        # Initialize Whisper model with optimization
        self._initialize_whisper_model()

        # Audio streaming parameters
        self.buffer_duration = 5.0  # 5 seconds of audio buffer
        self.process_interval = 2.0  # Process every 2 seconds
        self.min_speech_duration = 0.5  # Minimum speech to transcribe
        self.silence_threshold = 0.01  # Threshold for silence detection

        # Initialize audio buffer and processing state
        self.audio_buffer = deque(maxlen=int(16000 * self.buffer_duration))  # 16kHz * duration
        self.processing_lock = threading.Lock()
        self.last_process_time = time.time()
        self.speech_detected = False

        # Publishers
        self.transcription_pub = self.create_publisher(String, '/whisper/transcription', 10)
        self.confidence_pub = self.create_publisher(Float32, '/whisper/confidence', 10)
        self.is_speaking_pub = self.create_publisher(Bool, '/whisper/is_speaking', 10)

        # Subscriber
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )

        # Timer for continuous processing
        self.process_timer = self.create_timer(0.1, self._process_audio_continuously)

        # Performance metrics
        self.processing_times = deque(maxlen=100)
        self.transcription_count = 0

        self.get_logger().info('Streaming Whisper ASR node initialized')

    def _initialize_whisper_model(self):
        """Initialize Whisper model with streaming optimizations"""
        try:
            model_size = self.declare_parameter('model_size', 'small').value
            device = 'cuda' if torch.cuda.is_available() else 'cpu'

            self.get_logger().info(f'Loading Whisper {model_size} model on {device}...')
            self.model = whisper.load_model(model_size).to(device)

            # Use half precision for faster inference on GPU
            if device == 'cuda':
                self.model = self.model.half()

            self.get_logger().info('Whisper model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Whisper: {e}')
            raise

    def audio_callback(self, msg):
        """Handle incoming audio data"""
        try:
            # Convert audio data to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to buffer
            for sample in audio_data:
                self.audio_buffer.append(sample)

        except Exception as e:
            self.get_logger().error(f'Audio callback error: {e}')

    def _process_audio_continuously(self):
        """Continuously process audio buffer"""
        current_time = time.time()

        # Check if enough time has passed since last processing
        if current_time - self.last_process_time >= self.process_interval:
            with self.processing_lock:
                if len(self.audio_buffer) > 0:
                    # Convert buffer to numpy array
                    audio_array = np.array(list(self.audio_buffer))

                    # Check for speech activity
                    if self._detect_speech_activity(audio_array):
                        self.speech_detected = True
                        self._publish_speaking_status(True)

                        # Process for transcription
                        self._process_for_transcription(audio_array)

                        # Clear processed audio from buffer
                        # Keep some overlap for continuity
                        overlap_duration = 1.0  # 1 second overlap
                        overlap_samples = int(16000 * overlap_duration)
                        if len(self.audio_buffer) > overlap_samples:
                            # Remove processed samples but keep overlap
                            samples_to_remove = len(self.audio_buffer) - overlap_samples
                            for _ in range(samples_to_remove):
                                try:
                                    self.audio_buffer.popleft()
                                except IndexError:
                                    break
                    else:
                        if self.speech_detected:
                            self.speech_detected = False
                            self._publish_speaking_status(False)

                    self.last_process_time = current_time

    def _detect_speech_activity(self, audio_array):
        """Detect if speech is present in audio"""
        if len(audio_array) == 0:
            return False

        # Calculate energy-based speech detection
        energy = np.mean(audio_array ** 2)
        is_speech = energy > self.silence_threshold

        # Additional checks could include:
        # - Zero crossing rate
        # - Spectral features
        # - Machine learning-based VAD

        return is_speech

    def _process_for_transcription(self, audio_array):
        """Process audio for transcription"""
        try:
            start_time = time.time()

            # Perform transcription
            result = self.model.transcribe(
                audio_array,
                language='en',
                temperature=0.0,
                best_of=1,
                beam_size=5,
                patience=1.0
            )

            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)

            # Publish transcription if confidence is high enough
            if result and hasattr(result, 'text') and result.text.strip():
                confidence = self._estimate_confidence(result)

                if confidence > 0.5:  # Confidence threshold
                    transcription_msg = String()
                    transcription_msg.data = result.text.strip()
                    self.transcription_pub.publish(transcription_msg)

                    confidence_msg = Float32()
                    confidence_msg.data = float(confidence)
                    self.confidence_pub.publish(confidence_msg)

                    self.transcription_count += 1
                    self.get_logger().info(
                        f'Transcribed: "{result.text.strip()}" '
                        f'(confidence: {confidence:.3f}, time: {processing_time:.3f}s)'
                    )

        except Exception as e:
            self.get_logger().error(f'Transcription processing error: {e}')

    def _estimate_confidence(self, result):
        """Estimate confidence of transcription result"""
        if not result.segments:
            return 0.0

        # Calculate average confidence from segments
        confidences = []
        for segment in result.segments:
            if hasattr(segment, 'temperature'):
                # Lower temperature indicates higher confidence
                conf = max(0.0, 1.0 - segment.temperature)
                confidences.append(conf)

        if confidences:
            return sum(confidences) / len(confidences)
        else:
            # Fallback: use average log probability
            if hasattr(result, 'avg_logprob') and result.avg_logprob is not None:
                # Convert log probability to confidence (rough approximation)
                return max(0.0, min(1.0, (result.avg_logprob + 2.0) / 2.0))
            else:
                return 0.5  # Default confidence

    def _publish_speaking_status(self, is_speaking):
        """Publish speaking status"""
        msg = Bool()
        msg.data = is_speaking
        self.is_speaking_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StreamingWhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down streaming Whisper node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Activity Detection Integration

### Advanced VAD with Whisper

```python
# vad_whisper_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import numpy as np
import torch
import torchaudio
from collections import deque
import threading

class VADWhisperNode(Node):
    def __init__(self):
        super().__init__('vad_whisper_node')

        # Initialize components
        self._initialize_components()

        # Audio processing parameters
        self.sample_rate = 16000
        self.frame_size = 1024  # FFT frame size
        self.hop_length = 512   # Hop length for overlap
        self.vad_window_size = 1600  # 100ms at 16kHz
        self.speech_threshold = 0.3
        self.silence_duration = 1.0  # End speech after 1s of silence

        # Audio buffers
        self.audio_buffer = deque(maxlen=int(self.sample_rate * 10))  # 10s buffer
        self.speech_buffer = deque(maxlen=int(self.sample_rate * 5))  # 5s speech buffer
        self.silence_timer = 0.0
        self.in_speech = False

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )

        self.transcription_pub = self.create_publisher(String, '/vad_whisper/transcription', 10)
        self.speech_status_pub = self.create_publisher(Bool, '/vad_whisper/speech_status', 10)

        # Processing timer
        self.process_timer = self.create_timer(0.01, self._process_audio)

        self.get_logger().info('VAD-Whisper integrated node initialized')

    def _initialize_components(self):
        """Initialize Whisper model and VAD components"""
        try:
            # Initialize Whisper model
            model_size = self.declare_parameter('model_size', 'base').value
            device = 'cuda' if torch.cuda.is_available() else 'cpu'

            self.whisper_model = whisper.load_model(model_size).to(device)
            if device == 'cuda':
                self.whisper_model = self.whisper_model.half()

            # Initialize VAD model (could use Silero VAD or similar)
            # For this example, we'll implement a simple energy-based VAD
            self.vad_initialized = True
            self.get_logger().info('VAD-Whisper components initialized')

        except Exception as e:
            self.get_logger().error(f'Initialization error: {e}')
            self.vad_initialized = False

    def audio_callback(self, msg):
        """Process incoming audio with VAD"""
        try:
            # Convert to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to main buffer
            for sample in audio_data:
                self.audio_buffer.append(sample)

        except Exception as e:
            self.get_logger().error(f'Audio callback error: {e}')

    def _process_audio(self):
        """Process audio for VAD and transcription"""
        if not self.vad_initialized or len(self.audio_buffer) < self.vad_window_size:
            return

        # Get current audio chunk
        audio_chunk = np.array(list(self.audio_buffer)[-self.vad_window_size:])

        # Perform VAD
        is_speech = self._perform_vad(audio_chunk)

        if is_speech and not self.in_speech:
            # Start of speech detected
            self.in_speech = True
            self.speech_buffer.clear()
            self.silence_timer = 0.0
            self._publish_speech_status(True)
            self.get_logger().info('Speech detected - starting recording')

        elif is_speech and self.in_speech:
            # Continue recording speech
            for sample in audio_chunk:
                self.speech_buffer.append(sample)
            self.silence_timer = 0.0

        elif not is_speech and self.in_speech:
            # Potentially end of speech
            self.silence_timer += 0.01  # Timer increment based on 100Hz processing

            if self.silence_timer >= self.silence_duration:
                # End of speech detected - process for transcription
                self._end_speech_and_transcribe()
            else:
                # Still in silence period, continue monitoring
                for sample in audio_chunk:
                    self.speech_buffer.append(sample)

        # If not in speech, add to main buffer for VAD analysis
        elif not self.in_speech:
            # Only keep recent audio for VAD analysis
            pass

    def _perform_vad(self, audio_chunk):
        """Perform voice activity detection"""
        # Simple energy-based VAD
        energy = np.mean(audio_chunk ** 2)
        is_speech = energy > self.speech_threshold

        # Could be enhanced with:
        # - Zero-crossing rate
        # - Spectral features
        # - ML-based VAD models
        # - Pitch detection

        return is_speech

    def _end_speech_and_transcribe(self):
        """End speech recording and perform transcription"""
        if len(self.speech_buffer) > self.sample_rate * 0.5:  # At least 0.5 seconds
            # Convert speech buffer to numpy array
            speech_audio = np.array(list(self.speech_buffer))

            # Perform transcription in separate thread
            transcription_thread = threading.Thread(
                target=self._transcribe_speech,
                args=(speech_audio,)
            )
            transcription_thread.start()

        # Reset speech state
        self.in_speech = False
        self.silence_timer = 0.0
        self._publish_speech_status(False)
        self.get_logger().info('Speech ended - transcription queued')

    def _transcribe_speech(self, speech_audio):
        """Transcribe speech audio using Whisper"""
        try:
            result = self.whisper_model.transcribe(
                speech_audio,
                language='en',
                temperature=0.0,
                compression_ratio_threshold=2.0,
                logprob_threshold=-1.0
            )

            if result and result.text.strip():
                transcription_msg = String()
                transcription_msg.data = result.text.strip()
                self.transcription_pub.publish(transcription_msg)

                self.get_logger().info(f'Transcribed: "{result.text.strip()}"')

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

    def _publish_speech_status(self, is_speaking):
        """Publish speech status"""
        msg = Bool()
        msg.data = is_speaking
        self.speech_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VADWhisperIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VAD-Whisper node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Whisper Model Optimization

### TensorRT Optimization for Whisper

```python
# tensorrt_whisper.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import torch
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

class TensorRTWhisperNode(Node):
    def __init__(self):
        super().__init__('tensorrt_whisper_node')

        # Initialize TensorRT components
        self._initialize_tensorrt_whisper()

        # Audio processing
        self.audio_buffer = np.array([])
        self.processing_queue = []

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )

        self.transcription_pub = self.create_publisher(String, '/tensorrt_whisper/transcription', 10)

        self.get_logger().info('TensorRT-optimized Whisper node initialized')

    def _initialize_tensorrt_whisper(self):
        """Initialize TensorRT-optimized Whisper"""
        try:
            # Note: This is conceptual - actual TensorRT optimization of Whisper
            # would require more complex model conversion
            self.trt_available = True

            # Initialize CUDA context
            self.cuda_ctx = cuda.Device(0).make_context()

            # In practice, you would:
            # 1. Convert Whisper model to ONNX
            # 2. Build TensorRT engine from ONNX
            # 3. Load and optimize the engine

            self.get_logger().info('TensorRT Whisper optimization prepared')

        except Exception as e:
            self.get_logger().error(f'TensorRT initialization error: {e}')
            self.trt_available = False

    def audio_callback(self, msg):
        """Process audio with TensorRT-optimized Whisper"""
        try:
            # Convert audio data
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to processing queue
            self.processing_queue.append(audio_data)

            # Process in batches if TensorRT is available
            if self.trt_available and len(self.processing_queue) > 5:
                self._process_audio_batch_tensorrt()

        except Exception as e:
            self.get_logger().error(f'TensorRT audio processing error: {e}')

    def _process_audio_batch_tensorrt(self):
        """Process audio batch using TensorRT optimization"""
        # This would implement actual TensorRT inference
        # For now, we'll use regular Whisper as fallback
        if not self.processing_queue:
            return

        # Combine audio chunks
        combined_audio = np.concatenate(self.processing_queue)
        self.processing_queue.clear()

        # Perform transcription using regular Whisper (would be TensorRT in production)
        try:
            result = self.whisper_model.transcribe(combined_audio, language='en')
            if result and result.text.strip():
                transcription_msg = String()
                transcription_msg.data = result.text.strip()
                self.transcription_pub.publish(transcription_msg)
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TensorRTWhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-time Performance Optimization

### Optimized Real-time Processing

```python
# real_time_optimization.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from audio_common_msgs.msg import AudioData
import numpy as np
import threading
import queue
import time

class RealTimeWhisperNode(Node):
    def __init__(self):
        super().__init__('real_time_whisper_node')

        # Initialize Whisper model
        self._initialize_model()

        # Real-time processing components
        self.audio_queue = queue.Queue(maxsize=10)  # Audio input queue
        self.result_queue = queue.Queue(maxsize=5)  # Result output queue
        self.processing_thread = None
        self.running = True

        # Performance parameters
        self.target_latency = 0.5  # 500ms target latency
        self.processing_interval = 0.2  # Process every 200ms
        self.max_buffer_size = int(16000 * 3)  # 3 seconds max buffer

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio/input',
            self.audio_callback,
            10
        )

        self.transcription_pub = self.create_publisher(String, '/real_time/transcription', 10)
        self.latency_pub = self.create_publisher(Float32, '/real_time/latency', 10)

        # Start processing thread
        self.start_processing_thread()

        self.get_logger().info('Real-time Whisper node initialized')

    def _initialize_model(self):
        """Initialize Whisper model for real-time processing"""
        try:
            model_size = self.declare_parameter('model_size', 'small').value
            device = 'cuda' if torch.cuda.is_available() else 'cpu'

            self.model = whisper.load_model(model_size).to(device)
            if device == 'cuda':
                self.model = self.model.half()

            self.get_logger().info(f'Real-time Whisper model loaded: {model_size} on {device}')

        except Exception as e:
            self.get_logger().error(f'Model initialization error: {e}')
            raise

    def audio_callback(self, msg):
        """Handle incoming audio data"""
        try:
            # Convert audio to numpy array
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to processing queue with timestamp
            timestamp = time.time()
            try:
                self.audio_queue.put_nowait((audio_data, timestamp))
            except queue.Full:
                # Drop oldest if queue is full
                try:
                    self.audio_queue.get_nowait()
                    self.audio_queue.put_nowait((audio_data, timestamp))
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f'Audio callback error: {e}')

    def start_processing_thread(self):
        """Start real-time processing thread"""
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def _processing_loop(self):
        """Real-time processing loop"""
        audio_buffer = np.array([])

        while self.running:
            try:
                # Get audio from queue
                try:
                    audio_data, input_timestamp = self.audio_queue.get(timeout=0.01)

                    # Add to buffer
                    audio_buffer = np.concatenate([audio_buffer, audio_data])

                    # Limit buffer size
                    if len(audio_buffer) > self.max_buffer_size:
                        audio_buffer = audio_buffer[-self.max_buffer_size:]

                except queue.Empty:
                    # No audio available, continue loop
                    continue

                # Process if we have enough audio and enough time has passed
                current_time = time.time()
                if (len(audio_buffer) > self.sample_rate * 0.5 and  # At least 0.5s
                    current_time - getattr(self, 'last_process_time', 0) > self.processing_interval):

                    # Process audio
                    start_process_time = time.time()
                    result = self.model.transcribe(audio_buffer, language='en')
                    process_time = time.time() - start_process_time

                    if result and result.text.strip():
                        # Calculate total latency
                        total_latency = (time.time() - input_timestamp) + process_time

                        # Publish result
                        transcription_msg = String()
                        transcription_msg.data = result.text.strip()
                        self.transcription_pub.publish(transcription_msg)

                        # Publish latency
                        latency_msg = Float32()
                        latency_msg.data = float(total_latency)
                        self.latency_pub.publish(latency_msg)

                        self.get_logger().info(
                            f'Real-time transcription: "{result.text.strip()}" '
                            f'(latency: {total_latency:.3f}s, process: {process_time:.3f}s)'
                        )

                    self.last_process_time = current_time
                    audio_buffer = np.array([])  # Clear buffer after processing

            except Exception as e:
                self.get_logger().error(f'Real-time processing error: {e}')

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeWhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with VLA Pipeline

### Complete VLA Integration

```python
# vla_whisper_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import whisper
import torch
import numpy as np

class VLAWhisperIntegrationNode(Node):
    def __init__(self):
        super().__init__('vla_whisper_integration')

        # Initialize Whisper
        self._initialize_whisper()

        # Subscribe to Whisper transcriptions
        self.transcription_sub = self.create_subscription(
            String,
            '/whisper/transcription',
            self.transcription_callback,
            10
        )

        # Subscribe to other VLA components
        self.vision_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.vision_callback,
            10
        )

        # Publishers for VLA commands
        self.command_pub = self.create_publisher(String, '/vla/commands', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # VLA state management
        self.current_scene_objects = []
        self.last_transcription = ""
        self.command_history = []

        self.get_logger().info('VLA-Whisper integration initialized')

    def _initialize_whisper(self):
        """Initialize Whisper for VLA integration"""
        try:
            model_size = self.declare_parameter('model_size', 'medium').value
            device = 'cuda' if torch.cuda.is_available() else 'cpu'

            self.whisper_model = whisper.load_model(model_size).to(device)
            if device == 'cuda':
                self.whisper_model = self.whisper_model.half()

            self.get_logger().info('Whisper initialized for VLA integration')

        except Exception as e:
            self.get_logger().error(f'Whisper initialization error: {e}')
            raise

    def transcription_callback(self, msg):
        """Process Whisper transcription for VLA commands"""
        transcription = msg.data.lower().strip()

        if not transcription:
            return

        self.last_transcription = transcription

        # Parse command and generate VLA response
        vla_command = self._parse_natural_language_command(transcription)

        if vla_command:
            command_msg = String()
            command_msg.data = vla_command
            self.command_pub.publish(command_msg)

            self.command_history.append({
                'transcription': transcription,
                'command': vla_command,
                'timestamp': self.get_clock().now().to_msg()
            })

            self.get_logger().info(f'VLA command generated: {vla_command}')

    def _parse_natural_language_command(self, text):
        """Parse natural language into VLA commands"""
        # Command patterns for humanoid robot
        command_patterns = {
            'navigation': [
                ('go to', r'go to (.+)'),
                ('move to', r'move to (.+)'),
                ('navigate to', r'navigate to (.+)'),
                ('walk to', r'walk to (.+)'),
                ('go to the', r'go to the (.+)'),
            ],
            'manipulation': [
                ('pick up', r'pick up (.+)'),
                ('grasp', r'grasp (.+)'),
                ('take', r'take (.+)'),
                ('get', r'get (.+)'),
                ('place', r'place (.+)'),
                ('put', r'put (.+)'),
            ],
            'perception': [
                ('look at', r'look at (.+)'),
                ('see', r'see (.+)'),
                ('find', r'find (.+)'),
                ('locate', r'locate (.+)'),
                ('where is', r'where is (.+)'),
            ],
            'communication': [
                ('say', r'say (.+)'),
                ('speak', r'speak (.+)'),
                ('tell me', r'tell me (.+)'),
                ('hello', r'hello'),
                ('hi', r'hi'),
            ]
        }

        # Check for command patterns
        for command_type, patterns in command_patterns.items():
            for pattern_name, regex_pattern in patterns:
                import re
                match = re.search(regex_pattern, text)
                if match:
                    if command_type == 'navigation':
                        target_location = match.group(1)
                        return f'NAVIGATE_TO:{target_location.upper()}'
                    elif command_type == 'manipulation':
                        target_object = match.group(1)
                        return f'MANIPULATE:{target_object.upper()}'
                    elif command_type == 'perception':
                        target_object = match.group(1)
                        return f'PERCEIVE:{target_object.upper()}'
                    elif command_type == 'communication':
                        if pattern_name == 'hello' or pattern_name == 'hi':
                            return 'COMMUNICATE:HELLO'
                        else:
                            content = match.group(1)
                            return f'COMMUNICATE:SPEAK:{content.upper()}'

        # If no specific pattern matched, return as general command
        return f'GENERAL_COMMAND:{text.upper()}'

    def vision_callback(self, msg):
        """Process vision data to contextualize speech commands"""
        # Update current scene objects
        self.current_scene_objects = []
        for detection in msg.detections:
            if detection.results:
                for result in detection.results:
                    self.current_scene_objects.append({
                        'class': result.hypothesis.class_id,
                        'confidence': result.hypothesis.score,
                        'bbox': detection.bbox
                    })

    def get_contextual_command(self, transcription):
        """Generate context-aware commands based on current scene"""
        # This would integrate current visual perception with speech commands
        # For example: "Pick up the red ball" -> identify which red ball in scene
        pass

def main(args=None):
    rclpy.init(args=args)
    node = VLAWhisperIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Monitoring and Tuning

### Whisper Performance Monitoring

```python
# performance_monitoring.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image
import time
import threading
from collections import deque

class WhisperPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('whisper_performance_monitor')

        # Performance metrics
        self.latency_history = deque(maxlen=100)
        self.throughput_history = deque(maxlen=100)
        self.cpu_usage_history = deque(maxlen=100)
        self.gpu_usage_history = deque(maxlen=100)

        # Publishers for performance metrics
        self.latency_pub = self.create_publisher(Float32, '/whisper/latency', 10)
        self.throughput_pub = self.create_publisher(Float32, '/whisper/throughput', 10)
        self.cpu_usage_pub = self.create_publisher(Float32, '/whisper/cpu_usage', 10)
        self.gpu_usage_pub = self.create_publisher(Float32, '/whisper/gpu_usage', 10)
        self.buffer_size_pub = self.create_publisher(Int32, '/whisper/buffer_size', 10)

        # Performance monitoring timer
        self.performance_timer = self.create_timer(1.0, self.publish_performance_metrics)

        # Statistics
        self.total_transcriptions = 0
        self.total_processing_time = 0.0

        self.get_logger().info('Whisper performance monitor initialized')

    def publish_performance_metrics(self):
        """Publish current performance metrics"""
        if self.latency_history:
            avg_latency = sum(self.latency_history) / len(self.latency_history)
            latency_msg = Float32()
            latency_msg.data = avg_latency
            self.latency_pub.publish(latency_msg)

        if self.throughput_history:
            avg_throughput = sum(self.throughput_history) / len(self.throughput_history)
            throughput_msg = Float32()
            throughput_msg.data = avg_throughput
            self.throughput_pub.publish(throughput_msg)

        # Publish other metrics...
        # CPU usage, GPU usage, etc.

def monitor_whisper_performance():
    """Function to monitor Whisper node performance"""
    monitor = WhisperPerformanceMonitor()
    return monitor
```

## Best Practices

### Whisper Integration Best Practices

1. **Model Selection**: Choose the right model size for your performance requirements
2. **GPU Utilization**: Use GPU acceleration for real-time performance
3. **Buffer Management**: Properly manage audio buffers to minimize latency
4. **VAD Integration**: Use voice activity detection to reduce unnecessary processing
5. **Error Handling**: Implement robust error handling for audio processing
6. **Privacy**: Consider privacy implications of speech processing
7. **Localization**: Support multiple languages for international applications
8. **Adaptation**: Fine-tune models for specific robotic vocabularies

### Performance Optimization Tips

1. **Batch Processing**: Process audio in appropriate batch sizes
2. **Memory Management**: Efficiently manage GPU memory for continuous operation
3. **Threading**: Use proper threading to avoid blocking audio processing
4. **Quality of Service**: Configure appropriate QoS settings for real-time requirements
5. **Resource Monitoring**: Continuously monitor system resources
6. **Fallback Mechanisms**: Implement CPU fallbacks for GPU failures

## Troubleshooting

### Common Issues and Solutions

**Issue**: High latency in transcription
**Solution**: Use smaller model size, optimize audio buffering, use GPU acceleration

**Issue**: Poor transcription quality
**Solution**: Improve audio quality, adjust VAD thresholds, fine-tune model

**Issue**: GPU memory exhaustion
**Solution**: Use half-precision, reduce batch sizes, implement memory cleanup

**Issue**: Real-time performance issues
**Solution**: Optimize processing pipeline, use streaming approaches, reduce model complexity

## Next Steps

In the next chapter, we'll explore the NLP Parser component of our VLA pipeline, learning how to parse natural language commands into structured robot actions that can be understood by our humanoid robot's control systems.