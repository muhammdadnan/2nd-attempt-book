# NLP Parser

In this chapter, we'll explore the Natural Language Processing (NLP) parser component of our Vision-Language-Action (VLA) pipeline. The NLP parser transforms natural language commands into structured robot actions, enabling humanoid robots to understand and execute human instructions effectively.

## Understanding NLP for Robotics

### Natural Language Understanding in Robotics

Natural Language Processing for robotics involves converting human language into executable robot behaviors. Unlike traditional NLP tasks, robotic NLP must handle:

- **Spatial reasoning**: Understanding "go to the kitchen" or "pick up the red cup"
- **Temporal reasoning**: Processing "after you pick up the cup, go to the table"
- **Embodied understanding**: Connecting language to physical actions and objects
- **Context awareness**: Understanding commands based on current robot state and environment

### NLP Pipeline Architecture

```
Natural Language Input → Tokenization → Syntax Analysis → Semantic Parsing → Action Generation → Robot Commands
```

## Core NLP Components

### 1. Language Understanding Pipeline

```python
# nlp_parser.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
import spacy
import nltk
from nltk.tokenize import word_tokenize
from nltk.tag import pos_tag
from nltk.chunk import ne_chunk
import numpy as np
import re
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class ParsedCommand:
    """Structured representation of a parsed command"""
    action: str
    target_object: Optional[str] = None
    target_location: Optional[str] = None
    attributes: Dict[str, str] = None
    parameters: Dict[str, float] = None
    confidence: float = 1.0

class NLPParserNode(Node):
    def __init__(self):
        super().__init__('nlp_parser')

        # Initialize NLP components
        self._initialize_nlp_components()

        # Subscribe to natural language commands
        self.command_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.command_callback,
            10
        )

        # Subscribe to context information
        self.vision_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.vision_callback,
            10
        )

        self.perception_sub = self.create_subscription(
            String,
            '/perception_context',
            self.perception_callback,
            10
        )

        # Publishers for parsed commands
        self.action_pub = self.create_publisher(String, '/parsed_action', 10)
        self.navigation_pub = self.create_publisher(PoseStamped, '/navigation_goal', 10)
        self.manipulation_pub = self.create_publisher(JointState, '/manipulation_command', 10)

        # Context information
        self.known_objects = {}
        self.known_locations = {}
        self.robot_state = {}

        self.get_logger().info('NLP Parser node initialized')

    def _initialize_nlp_components(self):
        """Initialize NLP processing components"""
        try:
            # Load spaCy model
            self.nlp = spacy.load("en_core_web_sm")
            self.get_logger().info('spaCy model loaded successfully')
        except OSError:
            self.get_logger().warn('spaCy model not found, downloading...')
            import subprocess
            subprocess.run(['python', '-m', 'spacy', 'download', 'en_core_web_sm'])
            self.nlp = spacy.load("en_core_web_sm")

        # Define action vocabulary for humanoid robots
        self.action_vocab = {
            # Navigation actions
            'move': ['move', 'go', 'walk', 'navigate', 'drive', 'travel'],
            'goto': ['go to', 'move to', 'navigate to', 'walk to', 'head to'],
            'approach': ['approach', 'come to', 'get near', 'move toward'],

            # Manipulation actions
            'grasp': ['grasp', 'grab', 'take', 'pick up', 'catch', 'seize'],
            'release': ['release', 'drop', 'let go', 'put down', 'place'],
            'carry': ['carry', 'transport', 'move', 'bring'],

            # Perception actions
            'look': ['look', 'see', 'find', 'locate', 'search', 'detect'],
            'identify': ['identify', 'recognize', 'find', 'spot', 'notice'],
            'describe': ['describe', 'tell me about', 'what is', 'explain'],

            # Communication actions
            'speak': ['say', 'speak', 'tell', 'announce', 'state'],
            'listen': ['listen', 'hear', 'pay attention'],
            'respond': ['respond', 'answer', 'reply', 'react']
        }

        # Define spatial relations
        self.spatial_relations = {
            'near': ['near', 'close to', 'by', 'next to'],
            'in_front_of': ['in front of', 'before', 'ahead of'],
            'behind': ['behind', 'after', 'back of'],
            'left_of': ['left of', 'to the left of'],
            'right_of': ['right of', 'to the right of'],
            'on': ['on', 'on top of', 'above'],
            'under': ['under', 'below', 'beneath'],
            'inside': ['inside', 'within', 'in'],
            'outside': ['outside', 'out of', 'beyond']
        }

        # Define object attributes
        self.attribute_patterns = {
            'color': ['red', 'blue', 'green', 'yellow', 'black', 'white', 'gray', 'orange', 'purple', 'pink'],
            'size': ['big', 'small', 'large', 'tiny', 'huge', 'mini', 'tall', 'short', 'wide', 'narrow'],
            'shape': ['round', 'square', 'rectangular', 'circular', 'triangular'],
            'material': ['wooden', 'metal', 'plastic', 'glass', 'ceramic', 'fabric']
        }

        self.get_logger().info('NLP components initialized')

    def command_callback(self, msg):
        """Process natural language command"""
        command_text = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {command_text}')

        try:
            # Parse the command
            parsed_command = self.parse_command(command_text)

            if parsed_command:
                # Generate appropriate robot command based on parsed action
                self._generate_robot_command(parsed_command)

                # Log the parsed command
                self.get_logger().info(
                    f'Parsed command: {parsed_command.action} '
                    f'object: {parsed_command.target_object} '
                    f'location: {parsed_command.target_location} '
                    f'confidence: {parsed_command.confidence:.2f}'
                )

        except Exception as e:
            self.get_logger().error(f'Command parsing error: {e}')

    def parse_command(self, command_text: str) -> Optional[ParsedCommand]:
        """Parse natural language command into structured format"""
        try:
            # Preprocess command
            command_text = self._preprocess_command(command_text)

            # Use spaCy for linguistic analysis
            doc = self.nlp(command_text)

            # Extract action
            action = self._extract_action(doc)

            # Extract target object
            target_object = self._extract_target_object(doc)

            # Extract target location
            target_location = self._extract_target_location(doc)

            # Extract attributes and parameters
            attributes = self._extract_attributes(doc)
            parameters = self._extract_parameters(doc)

            # Calculate confidence based on parsing completeness
            confidence = self._calculate_parsing_confidence(
                action, target_object, target_location
            )

            return ParsedCommand(
                action=action,
                target_object=target_object,
                target_location=target_location,
                attributes=attributes,
                parameters=parameters,
                confidence=confidence
            )

        except Exception as e:
            self.get_logger().error(f'Command parsing error: {e}')
            return None

    def _preprocess_command(self, command: str) -> str:
        """Preprocess command text"""
        # Remove extra whitespace
        command = ' '.join(command.split())

        # Handle common contractions
        command = command.replace("i'm", "i am").replace("don't", "do not")

        # Handle common abbreviations
        command = command.replace("robot", "robot").replace("bot", "robot")

        return command

    def _extract_action(self, doc) -> str:
        """Extract action verb from command"""
        # Look for action patterns in the command
        command_text = doc.text.lower()

        # Check for compound actions first
        for action_type, patterns in self.action_vocab.items():
            for pattern in patterns:
                if pattern in command_text:
                    return action_type

        # Use spaCy for POS tagging to find main verbs
        for token in doc:
            if token.pos_ == "VERB" and token.dep_ == "ROOT":
                # Check if the root verb matches any action
                for action_type, verbs in self.action_vocab.items():
                    if token.lemma_ in [v.split()[0] for v in verbs if ' ' not in v]:
                        return action_type

        # Default action if no specific action found
        return "unknown"

    def _extract_target_object(self, doc) -> Optional[str]:
        """Extract target object from command"""
        command_text = doc.text.lower()

        # Look for object patterns
        object_keywords = ['the', 'a', 'an', 'some']
        action_keywords = [word for sublist in self.action_vocab.values() for word in sublist]

        # Extract potential objects
        potential_objects = []
        for token in doc:
            if (token.pos_ in ["NOUN", "PROPN"] and
                token.text.lower() not in action_keywords and
                token.text.lower() not in object_keywords):
                potential_objects.append(token.text)

        # Use context to determine target object
        if potential_objects:
            # For now, return the first potential object
            # In practice, this would use more sophisticated context analysis
            return potential_objects[0]

        # Look for objects after spatial relations
        for i, token in enumerate(doc):
            if token.text.lower() in [rel for rels in self.spatial_relations.values() for rel in rels]:
                # Look for object after the spatial relation
                for j in range(i + 1, len(doc)):
                    if doc[j].pos_ in ["NOUN", "PROPN"]:
                        return doc[j].text

        return None

    def _extract_target_location(self, doc) -> Optional[str]:
        """Extract target location from command"""
        command_text = doc.text.lower()

        # Look for location patterns
        for rel_type, relations in self.spatial_relations.items():
            for relation in relations:
                if relation in command_text:
                    # Extract text after the spatial relation
                    parts = command_text.split(relation)
                    if len(parts) > 1:
                        location_part = parts[1].strip()
                        # Extract location-related nouns
                        for token in self.nlp(location_part):
                            if token.pos_ in ["NOUN", "PROPN", "ADP"]:
                                return token.text

        # Look for named entities that might be locations
        for ent in doc.ents:
            if ent.label_ in ["GPE", "LOC", "FAC"]:  # Geopolitical entity, Location, Facility
                return ent.text

        return None

    def _extract_attributes(self, doc) -> Dict[str, str]:
        """Extract object attributes from command"""
        attributes = {}

        for token in doc:
            # Check for color attributes
            if token.text.lower() in self.attribute_patterns['color']:
                attributes['color'] = token.text.lower()

            # Check for size attributes
            elif token.text.lower() in self.attribute_patterns['size']:
                attributes['size'] = token.text.lower()

            # Check for shape attributes
            elif token.text.lower() in self.attribute_patterns['shape']:
                attributes['shape'] = token.text.lower()

            # Check for material attributes
            elif token.text.lower() in self.attribute_patterns['material']:
                attributes['material'] = token.text.lower()

        return attributes

    def _extract_parameters(self, doc) -> Dict[str, float]:
        """Extract numerical parameters from command"""
        parameters = {}

        for token in doc:
            if token.like_num:
                # Look for context around numbers
                # e.g., "go forward 2 meters" -> distance: 2.0
                if token.i > 0:
                    prev_token = doc[token.i - 1]
                    if prev_token.text.lower() in ['forward', 'backward', 'left', 'right']:
                        parameters['distance'] = float(token.text)
                    elif token.text.lower() in ['meters', 'meter', 'm']:
                        # Look for preceding number
                        if token.i > 0 and doc[token.i - 1].like_num:
                            parameters['distance'] = float(doc[token.i - 1].text)

        return parameters

    def _calculate_parsing_confidence(self, action: str, target_obj: str, target_loc: str) -> float:
        """Calculate confidence score for parsing result"""
        confidence = 0.0

        # Action presence increases confidence
        if action and action != "unknown":
            confidence += 0.4

        # Target object presence increases confidence
        if target_obj:
            confidence += 0.3

        # Target location presence increases confidence
        if target_loc:
            confidence += 0.3

        # Ensure confidence doesn't exceed 1.0
        return min(confidence, 1.0)

    def _generate_robot_command(self, parsed_command: ParsedCommand):
        """Generate appropriate robot command based on parsed action"""
        action_type = parsed_command.action

        if action_type == 'goto':
            self._generate_navigation_command(parsed_command)
        elif action_type in ['grasp', 'take', 'pick']:
            self._generate_manipulation_command(parsed_command)
        elif action_type in ['look', 'find', 'locate']:
            self._generate_perception_command(parsed_command)
        elif action_type in ['speak', 'say']:
            self._generate_communication_command(parsed_command)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')

    def _generate_navigation_command(self, parsed_command: ParsedCommand):
        """Generate navigation command"""
        # For now, we'll use a simple approach
        # In practice, this would interface with known locations or perception system
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # Set default position (in practice, this would come from location mapping)
        goal_msg.pose.position.x = 1.0  # Example position
        goal_msg.pose.position.y = 1.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.navigation_pub.publish(goal_msg)

        # Publish action command
        action_msg = String()
        action_msg.data = f"NAVIGATE_TO:{parsed_command.target_location or 'default_location'}"
        self.action_pub.publish(action_msg)

    def _generate_manipulation_command(self, parsed_command: ParsedCommand):
        """Generate manipulation command"""
        # In practice, this would interface with object detection and manipulation planning
        joints_msg = JointState()
        joints_msg.name = ['gripper_joint']  # Example joint
        joints_msg.position = [0.5]  # Example position
        joints_msg.header.stamp = self.get_clock().now().to_msg()

        self.manipulation_pub.publish(joints_msg)

        # Publish action command
        action_msg = String()
        action_msg.data = f"MANIPULATE:{parsed_command.target_object or 'unknown_object'}"
        self.action_pub.publish(action_msg)

    def _generate_perception_command(self, parsed_command: ParsedCommand):
        """Generate perception command"""
        # Publish action command for perception system
        action_msg = String()
        action_msg.data = f"PERCEIVE:{parsed_command.target_object or 'environment'}"
        self.action_pub.publish(action_msg)

    def _generate_communication_command(self, parsed_command: ParsedCommand):
        """Generate communication command"""
        # Extract what to say from the command
        command_text = parsed_command.original_command.lower()
        speak_keyword = 'say' if 'say' in command_text else 'tell' if 'tell' in command_text else None

        if speak_keyword:
            # Extract the text to speak
            parts = command_text.split(speak_keyword)
            if len(parts) > 1:
                text_to_speak = parts[1].strip().strip('"\'')
                if text_to_speak:
                    action_msg = String()
                    action_msg.data = f"SPEAK:{text_to_speak}"
                    self.action_pub.publish(action_msg)

    def vision_callback(self, msg):
        """Update known objects from vision system"""
        for detection in msg.detections:
            if detection.results:
                # In practice, this would update known objects database
                pass

    def perception_callback(self, msg):
        """Update context from perception system"""
        # Update robot's understanding of the environment
        pass

def main(args=None):
    rclpy.init(args=args)
    node = NLPParserNode()

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

## Advanced NLP Techniques

### Named Entity Recognition for Robotics

```python
# ner_robotics.py
import spacy
from typing import List, Dict, Tuple
from dataclasses import dataclass
import re

@dataclass
class RobotEntity:
    """Named entity relevant to robotics"""
    text: str
    label: str  # OBJECT, LOCATION, ACTION, ATTRIBUTE, etc.
    start: int
    end: int
    confidence: float

class RoboticsNER:
    def __init__(self, base_model="en_core_web_sm"):
        """Initialize NER for robotics domain"""
        self.nlp = spacy.load(base_model)

        # Add custom patterns for robotics entities
        self._add_robotics_patterns()

        # Robotics-specific gazetteers
        self.object_gazetteer = [
            'cup', 'bottle', 'box', 'chair', 'table', 'door', 'window',
            'ball', 'toy', 'book', 'phone', 'computer', 'keyboard',
            'mouse', 'monitor', 'laptop', 'tablet', 'remote'
        ]

        self.location_gazetteer = [
            'kitchen', 'living room', 'bedroom', 'bathroom', 'office',
            'hallway', 'garage', 'garden', 'dining room', 'study',
            'corridor', 'entrance', 'exit', 'front door', 'back door'
        ]

    def _add_robotics_patterns(self):
        """Add custom patterns for robotics entities"""
        from spacy.lang.en import English
        from spacy.pipeline import EntityRuler

        ruler = EntityRuler(self.nlp)

        # Define patterns for robotics-specific entities
        patterns = [
            # Objects
            {"label": "ROBOT_OBJECT", "pattern": [{"LOWER": {"IN": self.object_gazetteer}}]},
            # Locations
            {"label": "ROBOT_LOCATION", "pattern": [{"LOWER": {"IN": self.location_gazetteer}}]},
            # Actions
            {"label": "ROBOT_ACTION", "pattern": [{"LOWER": {"IN": ["navigate", "grasp", "manipulate", "perceive"]}}]},
            # Colors
            {"label": "COLOR", "pattern": [{"LOWER": {"IN": ["red", "blue", "green", "yellow", "black", "white", "gray"]}}]},
            # Sizes
            {"label": "SIZE", "pattern": [{"LOWER": {"IN": ["big", "small", "large", "tiny", "huge", "mini"]}}]},
        ]

        ruler.add_patterns(patterns)
        self.nlp.add_pipe(ruler, before="ner")

    def extract_entities(self, text: str) -> List[RobotEntity]:
        """Extract robotics-specific entities from text"""
        doc = self.nlp(text)
        entities = []

        for ent in doc.ents:
            robot_ent = RobotEntity(
                text=ent.text,
                label=ent.label_,
                start=ent.start_char,
                end=ent.end_char,
                confidence=0.8  # Default confidence
            )
            entities.append(robot_ent)

        # Also look for gazetteer matches not caught by spaCy
        entities.extend(self._extract_gazetteer_entities(text))

        return entities

    def _extract_gazetteer_entities(self, text: str) -> List[RobotEntity]:
        """Extract entities using gazetteers"""
        entities = []
        text_lower = text.lower()

        # Search for object entities
        for obj in self.object_gazetteer:
            for match in re.finditer(rf'\b{re.escape(obj)}\b', text_lower):
                entities.append(RobotEntity(
                    text=text[match.start():match.end()],
                    label="ROBOT_OBJECT",
                    start=match.start(),
                    end=match.end(),
                    confidence=0.7
                ))

        # Search for location entities
        for loc in self.location_gazetteer:
            for match in re.finditer(rf'\b{re.escape(loc)}\b', text_lower):
                entities.append(RobotEntity(
                    text=text[match.start():match.end()],
                    label="ROBOT_LOCATION",
                    start=match.start(),
                    end=match.end(),
                    confidence=0.7
                ))

        return entities

def main():
    ner = RoboticsNER()

    # Test with sample commands
    test_commands = [
        "Go to the kitchen and pick up the red cup",
        "Find the blue ball near the table",
        "Navigate to the living room and look for the black laptop",
        "Grasp the small green box on the shelf"
    ]

    for command in test_commands:
        print(f"\nCommand: {command}")
        entities = ner.extract_entities(command)
        for entity in entities:
            print(f"  {entity.label}: {entity.text} (confidence: {entity.confidence:.2f})")

if __name__ == "__main__":
    main()
```

### Dependency Parsing for Action Understanding

```python
# dependency_parser.py
import spacy
from typing import List, Dict, Tuple
from dataclasses import dataclass

@dataclass
class DependencyRelation:
    """Represents a dependency relation in the sentence"""
    governor: str
    dependent: str
    relation: str
    governor_pos: str
    dependent_pos: str

class DependencyParser:
    def __init__(self, model="en_core_web_sm"):
        self.nlp = spacy.load(model)

    def analyze_dependencies(self, text: str) -> List[DependencyRelation]:
        """Analyze dependency relations in the text"""
        doc = self.nlp(text)
        relations = []

        for token in doc:
            if token.head != token:  # Skip root
                relation = DependencyRelation(
                    governor=token.head.text,
                    dependent=token.text,
                    relation=token.dep_,
                    governor_pos=token.head.pos_,
                    dependent_pos=token.pos_
                )
                relations.append(relation)

        return relations

    def extract_action_triplets(self, text: str) -> List[Dict]:
        """Extract action triplets (subject-verb-object) from text"""
        doc = self.nlp(text)
        triplets = []

        for token in doc:
            if token.pos_ == "VERB":
                # Find subject
                subject = None
                for child in token.children:
                    if child.dep_ in ("nsubj", "nsubjpass"):
                        subject = self._get_span_text(child)
                        break

                # Find object
                obj = None
                for child in token.children:
                    if child.dep_ in ("dobj", "pobj", "attr"):
                        obj = self._get_span_text(child)
                        break

                # Find prepositional objects
                prep_obj = None
                for child in token.children:
                    if child.dep_ == "prep":
                        for grandchild in child.children:
                            if grandchild.dep_ in ("pobj", "pcomp"):
                                prep_obj = self._get_span_text(grandchild)
                                break

                if subject or obj or prep_obj:
                    triplet = {
                        'subject': subject,
                        'verb': token.lemma_,
                        'object': obj,
                        'prepositional_object': prep_obj,
                        'full_text': token.text
                    }
                    triplets.append(triplet)

        return triplets

    def _get_span_text(self, token) -> str:
        """Get full text of token including its children"""
        span_start = token.i
        span_end = token.i + 1

        # Include children that are part of the phrase
        children_indices = []
        for child in token.subtree:
            children_indices.append(child.i)

        if children_indices:
            span_start = min(children_indices)
            span_end = max(children_indices) + 1

        return " ".join([token.text for token in token.doc[span_start:span_end]])

    def extract_spatial_relations(self, text: str) -> List[Dict]:
        """Extract spatial relations from text"""
        doc = self.nlp(text)
        spatial_relations = []

        for token in doc:
            if token.pos_ == "ADP":  # Preposition
                # Look for prepositional phrases indicating spatial relations
                if token.text.lower() in ["near", "by", "next to", "in front of", "behind", "on", "under"]:
                    prep_phrase = self._get_span_text(token)

                    # Find the object of the preposition
                    prep_obj = None
                    for child in token.children:
                        if child.dep_ in ("pobj", "pcomp"):
                            prep_obj = self._get_span_text(child)
                            break

                    # Find what the preposition modifies
                    modified_by_prep = None
                    for child in token.head.children:
                        if child.dep_ == "prep" and child == token:
                            # Find the noun that the prep phrase modifies
                            for ancestor in token.ancestors:
                                if ancestor.pos_ in ("NOUN", "PROPN"):
                                    modified_by_prep = self._get_span_text(ancestor)
                                    break

                    if prep_obj:
                        spatial_rel = {
                            'relation': token.text.lower(),
                            'reference_object': modified_by_prep,
                            'target_object': prep_obj,
                            'full_phrase': prep_phrase
                        }
                        spatial_relations.append(spatial_rel)

        return spatial_relations

def main():
    parser = DependencyParser()

    test_sentences = [
        "Go to the kitchen and pick up the red cup",
        "Find the blue ball near the table",
        "The robot navigates to the living room",
        "Place the small box on the shelf"
    ]

    for sentence in test_sentences:
        print(f"\nSentence: {sentence}")

        # Extract action triplets
        triplets = parser.extract_action_triplets(sentence)
        print("Action Triplets:")
        for triplet in triplets:
            print(f"  Subject: {triplet['subject']}, Verb: {triplet['verb']}, Object: {triplet['object']}")

        # Extract spatial relations
        spatial_rels = parser.extract_spatial_relations(sentence)
        print("Spatial Relations:")
        for rel in spatial_rels:
            print(f"  {rel['relation']} - Reference: {rel['reference_object']}, Target: {rel['target_object']}")

if __name__ == "__main__":
    main()
```

## Context-Aware NLP Processing

### Context Integration with Perception

```python
# context_aware_parser.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import spacy
import numpy as np
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class PerceptualContext:
    """Current perceptual context for NLP processing"""
    detected_objects: List[Dict]
    robot_pose: Optional[PoseStamped]
    environment_map: Optional[np.ndarray]
    timestamp: float

class ContextAwareParserNode(Node):
    def __init__(self):
        super().__init__('context_aware_parser')

        # Initialize NLP components
        self.nlp = spacy.load("en_core_web_sm")

        # Initialize context
        self.current_context = PerceptualContext(
            detected_objects=[],
            robot_pose=None,
            environment_map=None,
            timestamp=0.0
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.command_callback,
            10
        )

        self.vision_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.vision_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # Publishers
        self.resolved_command_pub = self.create_publisher(String, '/resolved_command', 10)
        self.context_pub = self.create_publisher(String, '/nlp_context', 10)

        self.get_logger().info('Context-aware NLP parser initialized')

    def vision_callback(self, msg):
        """Update context with vision information"""
        detected_objects = []
        for detection in msg.detections:
            if detection.results:
                # Extract object information
                for result in detection.results:
                    obj_info = {
                        'class': result.hypothesis.class_id,
                        'confidence': result.hypothesis.score,
                        'bbox': detection.bbox,
                        'pose': detection.pose  # If available
                    }
                    detected_objects.append(obj_info)

        self.current_context.detected_objects = detected_objects
        self.current_context.timestamp = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f'Updated context with {len(detected_objects)} detected objects')

    def pose_callback(self, msg):
        """Update context with robot pose"""
        self.current_context.robot_pose = msg
        self.current_context.timestamp = self.get_clock().now().nanoseconds / 1e9

    def command_callback(self, msg):
        """Process command with context awareness"""
        command_text = msg.data
        self.get_logger().info(f'Received contextual command: {command_text}')

        try:
            # Resolve ambiguous references using context
            resolved_command = self.resolve_command_with_context(command_text)

            if resolved_command:
                # Publish resolved command
                resolved_msg = String()
                resolved_msg.data = resolved_command
                self.resolved_command_pub.publish(resolved_msg)

                self.get_logger().info(f'Resolved command: {resolved_command}')

        except Exception as e:
            self.get_logger().error(f'Context-aware parsing error: {e}')

    def resolve_command_with_context(self, command: str) -> Optional[str]:
        """Resolve command using perceptual context"""
        try:
            # Parse the command using dependency parsing
            doc = self.nlp(command)

            # Identify ambiguous references
            resolved_command = command

            # Look for pronouns or demonstratives that need resolution
            for token in doc:
                if token.text.lower() in ['it', 'them', 'that', 'those', 'the']:
                    # This is a simplified resolution - in practice, this would be more sophisticated
                    resolved_ref = self._resolve_reference(token.text.lower(), command)
                    if resolved_ref:
                        resolved_command = resolved_command.replace(token.text, resolved_ref)

            # Resolve spatial references using context
            resolved_command = self._resolve_spatial_references(resolved_command)

            # Resolve object references using detected objects
            resolved_command = self._resolve_object_references(resolved_command)

            return resolved_command

        except Exception as e:
            self.get_logger().error(f'Command resolution error: {e}')
            return None

    def _resolve_reference(self, pronoun: str, command: str) -> Optional[str]:
        """Resolve pronoun or demonstrative using context"""
        if not self.current_context.detected_objects:
            return None

        # Simple resolution based on most confident detection
        # In practice, this would use more sophisticated coreference resolution
        most_confident_obj = max(
            self.current_context.detected_objects,
            key=lambda obj: obj['confidence'],
            default=None
        )

        if most_confident_obj:
            return most_confident_obj['class']

        return None

    def _resolve_spatial_references(self, command: str) -> str:
        """Resolve spatial references using robot pose and environment"""
        # Example: "go there" -> "go to [specific location]"
        if 'there' in command.lower():
            # In practice, this would use the robot's perception of where "there" is
            # based on gaze direction, pointing gesture, etc.
            if self.current_context.robot_pose:
                # This is a simplified example
                target_x = self.current_context.robot_pose.pose.position.x + 1.0  # 1m ahead
                target_y = self.current_context.robot_pose.pose.position.y
                return command.lower().replace('there', f'to {target_x:.2f}, {target_y:.2f}')

        return command

    def _resolve_object_references(self, command: str) -> str:
        """Resolve object references using detected objects"""
        if not self.current_context.detected_objects:
            return command

        resolved_command = command

        # Look for generic object references and try to match with detected objects
        for obj_info in self.current_context.detected_objects:
            obj_class = obj_info['class']
            confidence = obj_info['confidence']

            # Only resolve if confidence is high enough
            if confidence > 0.7:
                # Replace generic references with specific object names
                # This is a simplified example
                if obj_class in command.lower():
                    # Already specific
                    continue
                elif f'the {obj_class}' in command.lower():
                    # Already specific
                    continue
                else:
                    # Look for generic references that might refer to this object
                    if 'it' in command.lower() and obj_class in self._get_possible_antecedents(command):
                        resolved_command = resolved_command.replace('it', obj_class)

        return resolved_command

    def _get_possible_antecedents(self, command: str) -> List[str]:
        """Get possible antecedents for pronoun resolution"""
        # This would implement more sophisticated coreference resolution
        # For now, return common object types
        return ['cup', 'ball', 'box', 'chair', 'table', 'object']

    def _infer_missing_information(self, parsed_command: Dict) -> Dict:
        """Infer missing information using context"""
        resolved_command = parsed_command.copy()

        # Infer location if not specified but context suggests one
        if not parsed_command.get('target_location') and self.current_context.robot_pose:
            # Use current robot location as reference
            resolved_command['inferred_location'] = {
                'x': self.current_context.robot_pose.pose.position.x,
                'y': self.current_context.robot_pose.pose.position.y,
                'frame': self.current_context.robot_pose.header.frame_id
            }

        # Infer object properties from context
        if parsed_command.get('target_object'):
            for detected_obj in self.current_context.detected_objects:
                if (detected_obj['class'].lower() in parsed_command['target_object'].lower() or
                    parsed_command['target_object'].lower() in detected_obj['class'].lower()):
                    # Add detected properties to the command
                    resolved_command['object_properties'] = {
                        'confidence': detected_obj['confidence'],
                        'bbox': detected_obj['bbox'],
                        'pose': detected_obj.get('pose')
                    }
                    break

        return resolved_command

def main(args=None):
    rclpy.init(args=args)
    node = ContextAwareParserNode()

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

## Grammar-Based Command Parsing

### Formal Grammar for Robot Commands

```python
# grammar_parser.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyparsing import *
import json
from typing import Dict, Any

class GrammarBasedParserNode(Node):
    def __init__(self):
        super().__init__('grammar_based_parser')

        # Define formal grammar for robot commands
        self._define_command_grammar()

        # Subscribe to commands
        self.command_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.command_callback,
            10
        )

        self.parsed_command_pub = self.create_publisher(String, '/parsed_command', 10)

        self.get_logger().info('Grammar-based parser initialized')

    def _define_command_grammar(self):
        """Define formal grammar for robot commands"""
        # Define basic tokens
        MOVE_VERBS = ['go', 'move', 'navigate', 'walk', 'drive', 'travel']
        GRASP_VERBS = ['grasp', 'grab', 'take', 'pick', 'hold', 'catch']
        LOOK_VERBS = ['look', 'see', 'find', 'locate', 'search', 'detect']
        SPEAK_VERBS = ['say', 'speak', 'tell', 'announce', 'state']

        # Basic elements
        ARTICLE = CaselessKeyword("the") | CaselessKeyword("a") | CaselessKeyword("an")
        PREPOSITION = CaselessKeyword("to") | CaselessKeyword("at") | CaselessKeyword("near") | \
                     CaselessKeyword("on") | CaselessKeyword("in") | CaselessKeyword("under") | \
                     CaselessKeyword("over") | CaselessKeyword("by") | CaselessKeyword("next to")

        # Action verbs
        MOVE_VERB = oneOf(" ".join(MOVE_VERBS), caseless=True)("action_type")
        GRASP_VERB = oneOf(" ".join(GRASP_VERBS), caseless=True)("action_type")
        LOOK_VERB = oneOf(" ".join(LOOK_VERBS), caseless=True)("action_type")
        SPEAK_VERB = oneOf(" ".join(SPEAK_VERBS), caseless=True)("action_type")

        ACTION_VERB = MOVE_VERB | GRASP_VERB | LOOK_VERB | SPEAK_VERB

        # Objects and locations
        OBJECT_NAME = Word(alphas)("object_name")
        LOCATION_NAME = Word(alphas)("location_name")

        # Colors and attributes
        COLOR = oneOf("red blue green yellow black white gray orange purple pink", caseless=True)("color")
        SIZE = oneOf("big small large tiny huge mini tall short wide narrow", caseless=True)("size")

        # Grammar patterns
        NAVIGATION_PATTERN = Optional(ARTICLE) + ACTION_VERB + PREPOSITION + Optional(ARTICLE) + LOCATION_NAME
        MANIPULATION_PATTERN = Optional(ARTICLE) + ACTION_VERB + Optional(COLOR) + Optional(SIZE) + Optional(ARTICLE) + OBJECT_NAME
        PERCEPTION_PATTERN = Optional(ARTICLE) + ACTION_VERB + Optional(COLOR) + Optional(SIZE) + Optional(ARTICLE) + OBJECT_NAME
        COMMUNICATION_PATTERN = Optional(ARTICLE) + ACTION_VERB + restOfLine("content")

        # Complete grammar
        self.grammar = NAVIGATION_PATTERN | MANIPULATION_PATTERN | PERCEPTION_PATTERN | COMMUNICATION_PATTERN

        self.get_logger().info('Command grammar defined')

    def command_callback(self, msg):
        """Parse command using formal grammar"""
        command_text = msg.data.strip()
        self.get_logger().info(f'Parsing command: {command_text}')

        try:
            # Parse using defined grammar
            parsed_result = self.grammar.parseString(command_text, parseAll=True)

            # Convert to structured format
            structured_command = self._convert_to_structured_command(parsed_result)

            # Publish structured command
            command_msg = String()
            command_msg.data = json.dumps(structured_command)
            self.parsed_command_pub.publish(command_msg)

            self.get_logger().info(f'Parsed command: {structured_command}')

        except ParseException as e:
            self.get_logger().error(f'Grammar parsing failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Command parsing error: {e}')

    def _convert_to_structured_command(self, parsed_result) -> Dict[str, Any]:
        """Convert parsed result to structured command"""
        command = {
            'action_type': getattr(parsed_result, 'action_type', 'unknown'),
            'object_name': getattr(parsed_result, 'object_name', None),
            'location_name': getattr(parsed_result, 'location_name', None),
            'color': getattr(parsed_result, 'color', None),
            'size': getattr(parsed_result, 'size', None),
            'content': getattr(parsed_result, 'content', None)
        }

        return command

def main(args=None):
    rclpy.init(args=args)
    node = GrammarBasedParserNode()

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

## Machine Learning-Based NLP Enhancement

### Neural Network Integration

```python
# ml_nlp_enhancement.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import torch
import torch.nn as nn
import transformers
from transformers import AutoTokenizer, AutoModelForSequenceClassification
import numpy as np
from typing import Dict, List, Optional

class MLNLPEnhancerNode(Node):
    def __init__(self):
        super().__init__('ml_nlp_enhancer')

        # Initialize ML components
        self._initialize_ml_components()

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.command_callback,
            10
        )

        self.vision_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.vision_callback,
            10
        )

        # Publishers
        self.enhanced_command_pub = self.create_publisher(String, '/enhanced_command', 10)

        self.get_logger().info('ML-enhanced NLP node initialized')

    def _initialize_ml_components(self):
        """Initialize machine learning NLP components"""
        try:
            # Check for GPU availability
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.get_logger().info(f'Using device: {self.device}')

            # Load pre-trained model for command classification
            model_name = "bert-base-uncased"
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)

            # For command classification, we'll create a custom classifier
            self.command_classifier = self._create_command_classifier()

            # Load into device
            self.command_classifier.to(self.device)
            self.command_classifier.eval()

            # Define command classes
            self.command_classes = [
                'navigation', 'manipulation', 'perception',
                'communication', 'idle', 'unknown'
            ]

            self.get_logger().info('ML NLP components initialized')

        except Exception as e:
            self.get_logger().error(f'ML component initialization error: {e}')
            raise

    def _create_command_classifier(self):
        """Create command classification neural network"""
        class CommandClassifier(nn.Module):
            def __init__(self, hidden_size=768, num_classes=6):
                super(CommandClassifier, self).__init__()
                self.bert = transformers.AutoModel.from_pretrained("bert-base-uncased")
                self.dropout = nn.Dropout(0.1)
                self.classifier = nn.Linear(hidden_size, num_classes)

            def forward(self, input_ids, attention_mask):
                outputs = self.bert(input_ids=input_ids, attention_mask=attention_mask)
                pooled_output = outputs.pooler_output
                output = self.dropout(pooled_output)
                return self.classifier(output)

        return CommandClassifier()

    def command_callback(self, msg):
        """Process command with ML enhancement"""
        command_text = msg.data
        self.get_logger().info(f'Processing ML-enhanced command: {command_text}')

        try:
            # Enhance command understanding using ML
            enhanced_command = self._enhance_command_understanding(command_text)

            if enhanced_command:
                # Publish enhanced command
                enhanced_msg = String()
                enhanced_msg.data = enhanced_command
                self.enhanced_command_pub.publish(enhanced_msg)

                self.get_logger().info(f'Enhanced command: {enhanced_command}')

        except Exception as e:
            self.get_logger().error(f'ML enhancement error: {e}')

    def _enhance_command_understanding(self, command: str) -> Optional[str]:
        """Enhance command understanding using ML models"""
        try:
            # Tokenize command
            inputs = self.tokenizer(
                command,
                return_tensors="pt",
                padding=True,
                truncation=True,
                max_length=128
            )

            # Move to device
            input_ids = inputs['input_ids'].to(self.device)
            attention_mask = inputs['attention_mask'].to(self.device)

            # Get classification
            with torch.no_grad():
                outputs = self.command_classifier(input_ids, attention_mask)
                predictions = torch.nn.functional.softmax(outputs, dim=-1)
                predicted_class_idx = torch.argmax(predictions, dim=-1).item()
                confidence = predictions[0][predicted_class_idx].item()

            # Get predicted command type
            predicted_command_type = self.command_classes[predicted_class_idx]

            # Enhance based on context and classification
            enhanced_result = {
                'original_command': command,
                'predicted_type': predicted_command_type,
                'confidence': confidence,
                'enhanced_interpretation': self._create_enhanced_interpretation(
                    command, predicted_command_type, confidence
                )
            }

            return json.dumps(enhanced_result)

        except Exception as e:
            self.get_logger().error(f'Command enhancement error: {e}')
            return None

    def _create_enhanced_interpretation(self, command: str, command_type: str, confidence: float) -> str:
        """Create enhanced interpretation based on ML classification"""
        if confidence < 0.7:
            return f"UNCERTAIN: {command} (type: {command_type}, confidence: {confidence:.2f})"

        # Create specific interpretations based on command type
        if command_type == 'navigation':
            return f"NAVIGATION_COMMAND: {command}"
        elif command_type == 'manipulation':
            return f"MANIPULATION_COMMAND: {command}"
        elif command_type == 'perception':
            return f"PERCEPTION_COMMAND: {command}"
        elif command_type == 'communication':
            return f"COMMUNICATION_COMMAND: {command}"
        else:
            return f"GENERAL_COMMAND: {command}"

    def vision_callback(self, msg):
        """Use vision data to enhance command understanding"""
        # Store detected objects for context-aware command enhancement
        detected_objects = []
        for detection in msg.detections:
            if detection.results:
                for result in detection.results:
                    detected_objects.append({
                        'class': result.hypothesis.class_id,
                        'confidence': result.hypothesis.score
                    })

        # This context could be used to disambiguate commands
        # For example, "pick up the cup" when multiple cups are detected
        self.detected_objects_context = detected_objects

def main(args=None):
    rclpy.init(args=args)
    node = MLNLPEnhancementNode()

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

## Performance Optimization

### Efficient NLP Processing

```python
# nlp_optimization.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
import time
import threading
from collections import deque
import queue

class OptimizedNLPNode(Node):
    def __init__(self):
        super().__init__('optimized_nlp_node')

        # Initialize optimized NLP components
        self._initialize_optimized_components()

        # Processing queue for batch processing
        self.processing_queue = queue.Queue(maxsize=100)
        self.batch_size = 5
        self.batch_timeout = 0.1  # 100ms timeout

        # Performance monitoring
        self.processing_times = deque(maxlen=100)
        self.throughput_history = deque(maxlen=100)

        # Subscribers and publishers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.command_callback,
            10
        )

        self.performance_pub = self.create_publisher(Float32, '/nlp_performance', 10)

        # Start processing thread
        self.processing_thread = threading.Thread(target=self._batch_processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Optimized NLP node initialized')

    def _initialize_optimized_components(self):
        """Initialize optimized NLP components"""
        try:
            # Use lightweight model for faster processing
            import spacy
            self.nlp = spacy.load("en_core_web_sm")  # Small model for speed

            # Cache frequently used patterns
            self.pattern_cache = {}

            # Initialize threading components
            self.processing_lock = threading.Lock()

            self.get_logger().info('Optimized NLP components initialized')

        except Exception as e:
            self.get_logger().error(f'Optimized NLP initialization error: {e}')
            raise

    def command_callback(self, msg):
        """Add command to processing queue"""
        try:
            # Add to processing queue with timestamp
            timestamp = time.time()
            self.processing_queue.put((msg.data, timestamp), block=False)

        except queue.Full:
            self.get_logger().warn('Processing queue full, dropping command')

    def _batch_processing_loop(self):
        """Process commands in batches for efficiency"""
        batch = []
        last_batch_time = time.time()

        while rclpy.ok():
            try:
                # Collect commands for batch
                while len(batch) < self.batch_size:
                    try:
                        command, timestamp = self.processing_queue.get(
                            timeout=max(0.0, self.batch_timeout - (time.time() - last_batch_time))
                        )
                        batch.append((command, timestamp))

                        # Check if we should process the batch
                        if len(batch) >= self.batch_size:
                            break

                    except queue.Empty:
                        break

                # Process batch if we have commands
                if batch:
                    self._process_batch(batch)
                    batch = []
                    last_batch_time = time.time()

            except Exception as e:
                self.get_logger().error(f'Batch processing error: {e}')

    def _process_batch(self, batch):
        """Process a batch of commands"""
        start_time = time.time()

        for command, timestamp in batch:
            try:
                # Process individual command
                result = self._optimized_parse_command(command)

                # Calculate and log performance
                processing_time = time.time() - start_time
                self.processing_times.append(processing_time)

            except Exception as e:
                self.get_logger().error(f'Batch command processing error: {e}')

        # Publish performance metrics
        avg_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0.0
        perf_msg = Float32()
        perf_msg.data = avg_time
        self.performance_pub.publish(perf_msg)

    def _optimized_parse_command(self, command: str):
        """Optimized command parsing"""
        # Use cached patterns if available
        if command in self.pattern_cache:
            return self.pattern_cache[command]

        # Perform parsing
        result = self._parse_command_internal(command)

        # Cache result (be mindful of memory usage)
        if len(self.pattern_cache) < 1000:  # Limit cache size
            self.pattern_cache[command] = result

        return result

    def _parse_command_internal(self, command: str):
        """Internal command parsing logic"""
        # Optimized parsing using spaCy
        doc = self.nlp(command)

        # Extract key information efficiently
        action = None
        target = None
        location = None

        for token in doc:
            if token.pos_ == "VERB" and token.dep_ == "ROOT":
                action = token.lemma_
            elif token.pos_ in ["NOUN", "PROPN"]:
                target = token.text
            elif token.text.lower() in ["kitchen", "living room", "bedroom"]:
                location = token.text

        return {
            'action': action,
            'target': target,
            'location': location,
            'confidence': 0.8  # Default confidence
        }

    def get_performance_metrics(self):
        """Get performance metrics"""
        if not self.processing_times:
            return {'avg_processing_time': 0.0, 'throughput': 0.0}

        avg_time = sum(self.processing_times) / len(self.processing_times)
        throughput = len(self.processing_times) / min(len(self.processing_times), 10)  # Commands per second

        return {
            'avg_processing_time': avg_time,
            'throughput': throughput
        }

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedNLPNode()

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

### Complete VLA NLP Integration

```python
# vla_nlp_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from builtin_interfaces.msg import Duration
import json
from typing import Dict, Any, List

class VLANLPIntegrationNode(Node):
    def __init__(self):
        super().__init__('vla_nlp_integration')

        # Initialize NLP components
        self._initialize_vla_nlp_components()

        # VLA pipeline subscribers
        self.natural_language_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.natural_language_callback,
            10
        )

        self.perception_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.perception_callback,
            10
        )

        # VLA pipeline publishers
        self.vla_command_pub = self.create_publisher(String, '/vla/command', 10)
        self.vla_context_pub = self.create_publisher(String, '/vla/context', 10)

        # VLA state management
        self.perception_context = {}
        self.command_history = []
        self.action_queue = []

        self.get_logger().info('VLA NLP Integration node initialized')

    def _initialize_vla_nlp_components(self):
        """Initialize VLA-specific NLP components"""
        try:
            import spacy
            self.nlp = spacy.load("en_core_web_sm")

            # VLA-specific action mappings
            self.vla_action_mappings = {
                'navigation': {
                    'patterns': ['go to', 'navigate to', 'move to', 'walk to'],
                    'output_format': 'NAVIGATION_GOAL:{location}'
                },
                'manipulation': {
                    'patterns': ['pick up', 'grasp', 'take', 'grab', 'place', 'put'],
                    'output_format': 'MANIPULATION_ACTION:{action}:{object}'
                },
                'perception': {
                    'patterns': ['look at', 'find', 'locate', 'detect', 'see'],
                    'output_format': 'PERCEPTION_REQUEST:{object}'
                },
                'communication': {
                    'patterns': ['say', 'speak', 'tell', 'announce'],
                    'output_format': 'COMMUNICATION_REQUEST:{content}'
                }
            }

            self.get_logger().info('VLA NLP components initialized')

        except Exception as e:
            self.get_logger().error(f'VLA NLP initialization error: {e}')
            raise

    def natural_language_callback(self, msg):
        """Process natural language command for VLA pipeline"""
        command_text = msg.data.strip()
        self.get_logger().info(f'VLA received command: {command_text}')

        try:
            # Parse command with VLA-specific understanding
            vla_command = self._parse_vla_command(command_text)

            if vla_command:
                # Publish to VLA pipeline
                vla_msg = String()
                vla_msg.data = json.dumps(vla_command)
                self.vla_command_pub.publish(vla_msg)

                # Update command history
                self.command_history.append({
                    'command': command_text,
                    'vla_command': vla_command,
                    'timestamp': self.get_clock().now().to_msg()
                })

                self.get_logger().info(f'VLA command published: {vla_command}')

        except Exception as e:
            self.get_logger().error(f'VLA command processing error: {e}')

    def perception_callback(self, msg):
        """Update VLA context with perception data"""
        # Update perception context for command resolution
        objects = []
        for detection in msg.detections:
            if detection.results:
                for result in detection.results:
                    objects.append({
                        'class': result.hypothesis.class_id,
                        'confidence': result.hypothesis.score,
                        'bbox': {
                            'center_x': detection.bbox.center.x,
                            'center_y': detection.bbox.center.y,
                            'size_x': detection.bbox.size_x,
                            'size_y': detection.bbox.size_y
                        }
                    })

        self.perception_context = {
            'objects': objects,
            'timestamp': self.get_clock().now().to_msg(),
            'frame_id': msg.header.frame_id
        }

        # Publish context update
        context_msg = String()
        context_msg.data = json.dumps(self.perception_context)
        self.vla_context_pub.publish(context_msg)

    def _parse_vla_command(self, command_text: str) -> Dict[str, Any]:
        """Parse command specifically for VLA pipeline"""
        # Use spaCy for linguistic analysis
        doc = self.nlp(command_text)

        # Identify command type
        command_type = self._identify_command_type(command_text)

        # Extract semantic components
        action = self._extract_action(doc)
        target_object = self._extract_target_object(doc)
        target_location = self._extract_target_location(doc)
        attributes = self._extract_attributes(doc)

        # Resolve references using perception context
        resolved_target = self._resolve_with_context(target_object)
        resolved_location = self._resolve_with_context(target_location)

        # Create VLA-compatible command structure
        vla_command = {
            'type': command_type,
            'action': action,
            'target_object': resolved_target,
            'target_location': resolved_location,
            'attributes': attributes,
            'context_timestamp': self.perception_context.get('timestamp'),
            'command_original': command_text,
            'confidence': self._calculate_command_confidence(command_text)
        }

        return vla_command

    def _identify_command_type(self, command: str) -> str:
        """Identify command type for VLA pipeline"""
        command_lower = command.lower()

        for cmd_type, config in self.vla_action_mappings.items():
            for pattern in config['patterns']:
                if pattern in command_lower:
                    return cmd_type

        return 'unknown'

    def _resolve_with_context(self, target: str) -> str:
        """Resolve target using perception context"""
        if not target or not self.perception_context:
            return target

        # Look for target in detected objects
        if 'objects' in self.perception_context:
            for obj in self.perception_context['objects']:
                if target.lower() in obj['class'].lower():
                    return obj['class']

        return target

    def _calculate_command_confidence(self, command: str) -> float:
        """Calculate confidence in command interpretation"""
        # Simple confidence calculation
        # In practice, this would use more sophisticated methods
        doc = self.nlp(command)

        # Count recognized entities and patterns
        recognized_elements = 0
        total_elements = len([token for token in doc if token.pos_ in ['NOUN', 'VERB', 'ADP']])

        for token in doc:
            if token.pos_ == 'VERB' or token.pos_ in ['NOUN', 'PROPN']:
                recognized_elements += 1

        confidence = recognized_elements / max(total_elements, 1) if total_elements > 0 else 0.0
        return min(confidence, 1.0)  # Clamp to [0, 1]

def main(args=None):
    rclpy.init(args=args)
    node = VLANLPIntegrationNode()

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

## Best Practices

### NLP Best Practices for Robotics

1. **Context Awareness**: Always consider environmental context for disambiguation
2. **Robust Parsing**: Handle various grammatical structures and informal language
3. **Error Recovery**: Implement graceful degradation for misunderstood commands
4. **Performance**: Optimize for real-time processing requirements
5. **Validation**: Validate parsed commands against robot capabilities
6. **Learning**: Incorporate feedback to improve parsing accuracy
7. **Privacy**: Consider privacy implications of speech processing
8. **Multilingual**: Support multiple languages for international applications

### Performance Optimization Tips

1. **Batch Processing**: Process commands in batches when possible
2. **Caching**: Cache frequently used parsing results
3. **Model Optimization**: Use optimized NLP models for speed
4. **Threading**: Use separate threads for NLP processing
5. **Memory Management**: Efficiently manage memory for continuous operation
6. **Profiling**: Monitor and optimize processing times
7. **Fallback Systems**: Implement alternative parsing methods
8. **Quality of Service**: Configure appropriate QoS for real-time requirements

## Troubleshooting

### Common Issues and Solutions

**Issue**: Poor command recognition
**Solution**: Improve microphone quality, use noise reduction, fine-tune NLP models

**Issue**: Slow processing times
**Solution**: Use lightweight models, optimize batch processing, leverage GPU acceleration

**Issue**: Ambiguous command interpretation
**Solution**: Improve context integration, implement disambiguation dialogs, use reinforcement learning

**Issue**: Context drift
**Solution**: Regularly update context, implement context validation, use temporal consistency checks

## Next Steps

In the next chapter, we'll explore the LLM Planner component, learning how to integrate large language models for high-level task planning and reasoning in humanoid robots. We'll see how to connect natural language understanding with sophisticated planning capabilities to enable robots to execute complex, multi-step tasks based on human instructions.