# LLM Planner

In this chapter, we'll explore how to integrate Large Language Models (LLMs) for high-level task planning and reasoning in humanoid robots. The LLM Planner serves as the cognitive layer that transforms natural language commands into structured task plans, enabling robots to understand complex, multi-step instructions and execute them intelligently.

## Understanding LLM-Based Planning

### The Role of LLMs in Robotics

Large Language Models bring several key capabilities to humanoid robotics:

- **Natural Language Understanding**: Interpreting complex human instructions
- **Task Decomposition**: Breaking down complex tasks into executable steps
- **Reasoning**: Applying logical reasoning to plan execution
- **Context Awareness**: Understanding situational context
- **Learning**: Adapting to new situations and preferences
- **Explainability**: Providing human-readable explanations of robot behavior

### LLM Planner Architecture

```
Natural Language → LLM Processing → Task Decomposition → Plan Validation → Action Sequence → Robot Execution
Command         (Understanding)   (Planning)         (Verification)   (Generation)   (Execution)
```

## LLM Integration Fundamentals

### LLM Selection and Configuration

```python
# llm_planner.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import openai
import anthropic
import json
import asyncio
from typing import Dict, List, Optional, Any
import logging

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # Initialize LLM components
        self._initialize_llm_components()

        # Subscribers for natural language commands
        self.natural_language_sub = self.create_subscription(
            String,
            '/natural_language/command',
            self.natural_language_callback,
            10
        )

        # Publishers for plans and status
        self.plan_pub = self.create_publisher(String, '/generated_plan', 10)
        self.status_pub = self.create_publisher(String, '/llm_status', 10)
        self.action_pub = self.create_publisher(String, '/planned_action', 10)

        # Robot state and capability information
        self.robot_capabilities = self._get_robot_capabilities()
        self.environment_context = {}
        self.current_plan = None
        self.plan_execution_status = 'idle'

        self.get_logger().info('LLM Planner node initialized')

    def _initialize_llm_components(self):
        """Initialize LLM components and configurations"""
        try:
            # Load LLM configuration
            self.llm_provider = self.declare_parameter('llm_provider', 'openai').value
            self.model_name = self.declare_parameter('model_name', 'gpt-4-turbo').value
            self.temperature = self.declare_parameter('temperature', 0.3).value
            self.max_tokens = self.declare_parameter('max_tokens', 1000).value

            # Initialize based on provider
            if self.llm_provider == 'openai':
                self._initialize_openai()
            elif self.llm_provider == 'anthropic':
                self._initialize_anthropic()
            elif self.llm_provider == 'local':
                self._initialize_local_llm()
            else:
                raise ValueError(f'Unsupported LLM provider: {self.llm_provider}')

            # Planning parameters
            self.max_retries = 3
            self.timeout_seconds = 30.0

            self.get_logger().info(f'LLM Planner initialized with {self.llm_provider} ({self.model_name})')

        except Exception as e:
            self.get_logger().error(f'LLM initialization error: {e}')
            raise

    def _initialize_openai(self):
        """Initialize OpenAI client"""
        api_key = self.declare_parameter('openai_api_key', '').value
        if not api_key:
            raise ValueError('OpenAI API key not provided')

        openai.api_key = api_key
        self.client = openai.OpenAI(api_key=api_key)

    def _initialize_anthropic(self):
        """Initialize Anthropic client"""
        api_key = self.declare_parameter('anthropic_api_key', '').value
        if not api_key:
            raise ValueError('Anthropic API key not provided')

        self.client = anthropic.Anthropic(api_key=api_key)

    def _initialize_local_llm(self):
        """Initialize local LLM (e.g., Ollama, local transformers)"""
        # For local models, we might use transformers or Ollama
        try:
            from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
            import torch

            model_name = self.declare_parameter('local_model_name', 'microsoft/DialoGPT-medium').value
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.model = AutoModelForCausalLM.from_pretrained(model_name)
            self.generator = pipeline('text-generation', model=self.model, tokenizer=self.tokenizer)

            self.get_logger().info(f'Local LLM initialized: {model_name}')

        except ImportError:
            self.get_logger().error('Transformers library not available for local LLM')
            raise

    def _get_robot_capabilities(self) -> Dict[str, Any]:
        """Define robot capabilities for LLM planning"""
        return {
            'navigation': {
                'capabilities': ['move_to', 'navigate', 'go_to'],
                'areas': ['kitchen', 'living_room', 'bedroom', 'office', 'hallway'],
                'constraints': {
                    'max_speed': 0.5,  # m/s
                    'min_turn_radius': 0.3,  # meters
                    'obstacle_avoidance': True
                }
            },
            'manipulation': {
                'capabilities': ['grasp', 'release', 'pick_up', 'place'],
                'reach': {
                    'min_height': 0.2,  # meters
                    'max_height': 1.5,
                    'max_reach': 0.8
                },
                'gripper': {
                    'max_load': 2.0,  # kg
                    'max_opening': 0.1,  # meters
                    'types': ['parallel', 'spherical']
                }
            },
            'perception': {
                'capabilities': ['detect_objects', 'recognize_faces', 'measure_distance'],
                'sensors': ['camera', 'lidar', 'imu'],
                'range': {
                    'visual': 5.0,  # meters
                    'lidar': 10.0
                }
            },
            'communication': {
                'capabilities': ['speak', 'listen', 'gesture'],
                'languages': ['English', 'Spanish', 'French'],
                'output_methods': ['speech', 'text_display']
            }
        }

    def natural_language_callback(self, msg):
        """Process natural language command using LLM"""
        command_text = msg.data.strip()
        self.get_logger().info(f'Processing LLM command: {command_text}')

        try:
            # Generate plan using LLM
            plan = self._generate_plan_with_llm(command_text)

            if plan:
                # Validate the plan
                validated_plan = self._validate_plan(plan)

                if validated_plan:
                    # Publish the plan
                    plan_msg = String()
                    plan_msg.data = json.dumps(validated_plan)
                    self.plan_pub.publish(plan_msg)

                    self.current_plan = validated_plan
                    self.plan_execution_status = 'ready'

                    self.get_logger().info(f'Plan generated with {len(validated_plan["steps"])} steps')

                    # Execute plan automatically if configured
                    if self.declare_parameter('auto_execute', True).value:
                        self._execute_plan(validated_plan)

                else:
                    self.get_logger().warn('Generated plan failed validation')

            else:
                self.get_logger().error('LLM failed to generate plan')

        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            self._publish_status('error', str(e))

    def _generate_plan_with_llm(self, command: str) -> Optional[Dict[str, Any]]:
        """Generate task plan using LLM"""
        try:
            # Create system prompt with robot context
            system_prompt = self._create_system_prompt()

            # Create user prompt with command
            user_prompt = self._create_user_prompt(command)

            # Generate response based on provider
            if self.llm_provider == 'openai':
                response = self._generate_with_openai(system_prompt, user_prompt)
            elif self.llm_provider == 'anthropic':
                response = self._generate_with_anthropic(system_prompt, user_prompt)
            else:
                response = self._generate_with_local(system_prompt, user_prompt)

            # Parse and validate response
            plan = self._parse_llm_response(response)

            return plan

        except Exception as e:
            self.get_logger().error(f'LLM plan generation error: {e}')
            return None

    def _create_system_prompt(self) -> str:
        """Create system prompt for LLM"""
        return f"""
        You are an AI planning assistant for a humanoid robot. Your role is to convert natural language commands into structured task plans that the robot can execute.

        Robot Capabilities:
        - Navigation: {self.robot_capabilities['navigation']['capabilities']}
        - Manipulation: {self.robot_capabilities['manipulation']['capabilities']}
        - Perception: {self.robot_capabilities['perception']['capabilities']}
        - Communication: {self.robot_capabilities['communication']['capabilities']}

        Robot Constraints:
        - Navigation: Max speed {self.robot_capabilities['navigation']['constraints']['max_speed']} m/s
        - Manipulation: Max load {self.robot_capabilities['manipulation']['gripper']['max_load']} kg
        - Reach: Min height {self.robot_capabilities['manipulation']['reach']['min_height']}m, Max height {self.robot_capabilities['manipulation']['reach']['max_height']}m

        Available Areas: {', '.join(self.robot_capabilities['navigation']['areas'])}

        Response Format:
        {{
            "task": "description of the overall task",
            "steps": [
                {{
                    "id": integer,
                    "type": "navigation|manipulation|perception|communication",
                    "action": "specific action to perform",
                    "parameters": {{"param1": "value1", ...}},
                    "description": "human-readable description",
                    "success_criteria": "criteria for success",
                    "dependencies": [list of step IDs that must complete first]
                }}
            ],
            "estimated_duration": float_seconds,
            "confidence": 0.0-1.0
        }}

        Always return valid JSON. Be specific about locations, objects, and actions. Consider robot capabilities and constraints.
        """

    def _create_user_prompt(self, command: str) -> str:
        """Create user prompt with command"""
        return f"""
        Command: {command}

        Please generate a detailed task plan that the humanoid robot can execute to fulfill this command. Consider the robot's capabilities, current environment, and any implicit requirements.

        Current Environment Context:
        - Detected objects: {self.environment_context.get('detected_objects', 'None')}
        - Robot location: {self.environment_context.get('robot_location', 'Unknown')}
        - Available tools: {self.environment_context.get('available_tools', 'Standard gripper')}

        Generate the plan in the specified JSON format.
        """

    def _generate_with_openai(self, system_prompt: str, user_prompt: str) -> str:
        """Generate response using OpenAI"""
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            response_format={"type": "json_object"}
        )

        return response.choices[0].message.content

    def _generate_with_anthropic(self, system_prompt: str, user_prompt: str) -> str:
        """Generate response using Anthropic"""
        response = self.client.messages.create(
            model=self.model_name,
            system=system_prompt,
            messages=[
                {"role": "user", "content": user_prompt}
            ],
            max_tokens=self.max_tokens,
            temperature=self.temperature
        )

        return response.content[0].text

    def _generate_with_local(self, system_prompt: str, user_prompt: str) -> str:
        """Generate response using local model"""
        prompt = f"{system_prompt}\n\n{user_prompt}\n\nResponse:"

        # Generate text
        outputs = self.generator(
            prompt,
            max_length=len(prompt.split()) + 200,
            num_return_sequences=1,
            temperature=self.temperature,
            pad_token_id=self.tokenizer.eos_token_id
        )

        return outputs[0]['generated_text'][len(prompt):]

    def _parse_llm_response(self, response_text: str) -> Optional[Dict[str, Any]]:
        """Parse and validate LLM response"""
        try:
            # Extract JSON from response if wrapped in text
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1

            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                plan = json.loads(json_str)

                # Validate required fields
                required_fields = ['task', 'steps', 'estimated_duration', 'confidence']
                if all(field in plan for field in required_fields):
                    return plan
                else:
                    self.get_logger().error('LLM response missing required fields')
                    return None

            else:
                self.get_logger().error('No JSON found in LLM response')
                return None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parsing error: {e}')
            self.get_logger().debug(f'Response text: {response_text}')
            return None
        except Exception as e:
            self.get_logger().error(f'LLM response parsing error: {e}')
            return None

    def _validate_plan(self, plan: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Validate generated plan against robot capabilities"""
        try:
            # Check if all actions are supported
            for step in plan.get('steps', []):
                action_type = step.get('type')
                action = step.get('action')

                if action_type not in self.robot_capabilities:
                    self.get_logger().warn(f'Unsupported action type: {action_type}')
                    return None

                # Validate specific actions
                if action_type == 'navigation':
                    if action not in self.robot_capabilities['navigation']['capabilities']:
                        self.get_logger().warn(f'Unsupported navigation action: {action}')
                        return None
                elif action_type == 'manipulation':
                    if action not in self.robot_capabilities['manipulation']['capabilities']:
                        self.get_logger().warn(f'Unsupported manipulation action: {action}')
                        return None
                elif action_type == 'perception':
                    if action not in self.robot_capabilities['perception']['capabilities']:
                        self.get_logger().warn(f'Unsupported perception action: {action}')
                        return None
                elif action_type == 'communication':
                    if action not in self.robot_capabilities['communication']['capabilities']:
                        self.get_logger().warn(f'Unsupported communication action: {action}')
                        return None

            # Validate dependencies
            step_ids = {step['id'] for step in plan.get('steps', [])}
            for step in plan.get('steps', []):
                deps = step.get('dependencies', [])
                for dep_id in deps:
                    if dep_id not in step_ids:
                        self.get_logger().warn(f'Invalid dependency: {dep_id}')
                        return None

            # Add validation metadata
            plan['validated_at'] = self.get_clock().now().seconds_nanoseconds()
            plan['validation_passed'] = True

            return plan

        except Exception as e:
            self.get_logger().error(f'Plan validation error: {e}')
            return None

    def _execute_plan(self, plan: Dict[str, Any]):
        """Execute the validated plan"""
        self.plan_execution_status = 'executing'
        self.get_logger().info(f'Executing plan: {plan["task"]}')

        # Execute steps sequentially or based on dependencies
        for step in plan['steps']:
            self._execute_plan_step(step)

        self.plan_execution_status = 'completed'
        self._publish_status('completed', f'Plan completed: {plan["task"]}')

    def _execute_plan_step(self, step: Dict[str, Any]):
        """Execute a single plan step"""
        try:
            step_type = step['type']
            action = step['action']
            params = step.get('parameters', {})

            self.get_logger().info(f'Executing step: {action} ({step_type})')

            # Create action command
            action_msg = String()
            action_msg.data = json.dumps({
                'type': step_type,
                'action': action,
                'parameters': params,
                'step_id': step['id']
            })

            self.action_pub.publish(action_msg)

            # Wait for completion or timeout
            # In a real implementation, this would wait for action completion feedback
            import time
            time.sleep(1)  # Placeholder for actual execution

            self.get_logger().info(f'Step completed: {action}')

        except Exception as e:
            self.get_logger().error(f'Step execution error: {e}')
            self._publish_status('error', f'Step execution failed: {e}')

    def _publish_status(self, status: str, message: str = ""):
        """Publish status message"""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': status,
            'message': message,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        })
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

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

## Advanced Planning Techniques

### Hierarchical Task Planning

```python
# hierarchical_planning.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import json
from typing import Dict, List, Optional
from dataclasses import dataclass
from enum import Enum

class TaskPriority(Enum):
    CRITICAL = 1
    HIGH = 2
    MEDIUM = 3
    LOW = 4

@dataclass
class TaskStep:
    id: int
    type: str
    action: str
    parameters: Dict[str, any]
    description: str
    priority: TaskPriority
    dependencies: List[int]
    estimated_duration: float
    success_criteria: str

@dataclass
class TaskPlan:
    id: str
    task_description: str
    steps: List[TaskStep]
    estimated_duration: float
    confidence: float
    created_at: float

class HierarchicalPlannerNode(Node):
    def __init__(self):
        super().__init__('hierarchical_planner')

        # Initialize components
        self._initialize_planning_components()

        # Subscribers and publishers
        self.high_level_command_sub = self.create_subscription(
            String,
            '/high_level_command',
            self.high_level_command_callback,
            10
        )

        self.task_plan_pub = self.create_publisher(String, '/task_plan', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)

        # Task management
        self.active_tasks = {}
        self.completed_tasks = []
        self.failed_tasks = []

        self.get_logger().info('Hierarchical Planner initialized')

    def _initialize_planning_components(self):
        """Initialize hierarchical planning components"""
        # Define task templates for common operations
        self.task_templates = {
            'fetch_item': {
                'description': 'Fetch an item from location A to location B',
                'subtasks': [
                    {'type': 'navigation', 'action': 'navigate_to', 'priority': TaskPriority.HIGH},
                    {'type': 'perception', 'action': 'detect_object', 'priority': TaskPriority.HIGH},
                    {'type': 'manipulation', 'action': 'grasp_object', 'priority': TaskPriority.CRITICAL},
                    {'type': 'navigation', 'action': 'return_to', 'priority': TaskPriority.MEDIUM},
                    {'type': 'manipulation', 'action': 'place_object', 'priority': TaskPriority.HIGH}
                ]
            },
            'clean_area': {
                'description': 'Clean a specified area',
                'subtasks': [
                    {'type': 'navigation', 'action': 'navigate_to_area', 'priority': TaskPriority.HIGH},
                    {'type': 'perception', 'action': 'detect_obstacles', 'priority': TaskPriority.HIGH},
                    {'type': 'manipulation', 'action': 'clear_obstacles', 'priority': TaskPriority.CRITICAL},
                    {'type': 'navigation', 'action': 'navigate_to_clean_area', 'priority': TaskPriority.MEDIUM}
                ]
            },
            'greet_visitor': {
                'description': 'Greet a visitor at the door',
                'subtasks': [
                    {'type': 'navigation', 'action': 'navigate_to_door', 'priority': TaskPriority.HIGH},
                    {'type': 'perception', 'action': 'detect_person', 'priority': TaskPriority.CRITICAL},
                    {'type': 'communication', 'action': 'greet_person', 'priority': TaskPriority.HIGH},
                    {'type': 'perception', 'action': 'recognize_face', 'priority': TaskPriority.MEDIUM}
                ]
            }
        }

        self.get_logger().info('Task templates loaded')

    def high_level_command_callback(self, msg):
        """Process high-level command and decompose into subtasks"""
        command = msg.data

        try:
            # Determine task type from command
            task_type = self._identify_task_type(command)

            if task_type in self.task_templates:
                # Generate hierarchical plan
                plan = self._generate_hierarchical_plan(task_type, command)
                if plan:
                    self._publish_task_plan(plan)
            else:
                # Use LLM for unknown task types
                plan = self._generate_dynamic_plan(command)
                if plan:
                    self._publish_task_plan(plan)

        except Exception as e:
            self.get_logger().error(f'Hierarchical planning error: {e}')

    def _identify_task_type(self, command: str) -> Optional[str]:
        """Identify task type from command"""
        command_lower = command.lower()

        for task_type, template in self.task_templates.items():
            if any(keyword in command_lower for keyword in [
                'fetch', 'get', 'bring', 'pick up', 'retrieve'
            ]) and task_type == 'fetch_item':
                return 'fetch_item'
            elif any(keyword in command_lower for keyword in [
                'clean', 'tidy', 'organize', 'clear'
            ]) and task_type == 'clean_area':
                return 'clean_area'
            elif any(keyword in command_lower for keyword in [
                'greet', 'welcome', 'hello', 'visitor', 'guest'
            ]) and task_type == 'greet_visitor':
                return 'greet_visitor'

        return None

    def _generate_hierarchical_plan(self, task_type: str, command: str) -> Optional[TaskPlan]:
        """Generate hierarchical plan for known task type"""
        template = self.task_templates[task_type]

        # Create subtasks based on template
        steps = []
        for i, subtask_template in enumerate(template['subtasks']):
            step = TaskStep(
                id=i,
                type=subtask_template['type'],
                action=subtask_template['action'],
                parameters=self._extract_parameters(command, subtask_template),
                description=f"{subtask_template['type'].capitalize()} - {subtask_template['action']}",
                priority=subtask_template['priority'],
                dependencies=self._determine_dependencies(i, subtask_template),
                estimated_duration=self._estimate_duration(subtask_template),
                success_criteria=self._define_success_criteria(subtask_template)
            )
            steps.append(step)

        # Calculate total estimated duration
        total_duration = sum(step.estimated_duration for step in steps)

        plan = TaskPlan(
            id=f"{task_type}_{self.get_clock().now().nanoseconds}",
            task_description=f"{template['description']}: {command}",
            steps=steps,
            estimated_duration=total_duration,
            confidence=0.9,  # High confidence for templated tasks
            created_at=self.get_clock().now().nanoseconds / 1e9
        )

        return plan

    def _extract_parameters(self, command: str, subtask_template: Dict) -> Dict[str, any]:
        """Extract parameters for subtask from command"""
        params = {}

        # Extract object names, locations, etc. from command
        # This would use NLP parsing to extract relevant information
        if 'object' in subtask_template['action']:
            # Look for object references in command
            import re
            object_patterns = [
                r'pick up the (\w+)',
                r'grasp the (\w+)',
                r'detect the (\w+)',
                r'find the (\w+)'
            ]

            for pattern in object_patterns:
                match = re.search(pattern, command, re.IGNORECASE)
                if match:
                    params['target_object'] = match.group(1)
                    break

        # Extract location information
        location_patterns = [
            r'to the (\w+)',
            r'at the (\w+)',
            r'near the (\w+)',
            r'in the (\w+)'
        ]

        for pattern in location_patterns:
            match = re.search(pattern, command, re.IGNORECASE)
            if match:
                params['target_location'] = match.group(1)
                break

        return params

    def _determine_dependencies(self, step_index: int, subtask_template: Dict) -> List[int]:
        """Determine dependencies for a subtask"""
        # Basic dependency rules
        dependencies = []

        # Most subtasks depend on previous step
        if step_index > 0:
            dependencies.append(step_index - 1)

        # Specific dependency rules
        if 'place' in subtask_template['action']:
            # Place action depends on grasp action
            dependencies.extend([i for i in range(step_index)
                               if 'grasp' in self.task_templates.get('fetch_item', {}).get('subtasks', [{}])[i].get('action', '')])

        return dependencies

    def _estimate_duration(self, subtask_template: Dict) -> float:
        """Estimate duration for subtask"""
        base_durations = {
            'navigation': 5.0,  # seconds
            'manipulation': 3.0,
            'perception': 2.0,
            'communication': 1.0
        }

        return base_durations.get(subtask_template['type'], 3.0)

    def _define_success_criteria(self, subtask_template: Dict) -> str:
        """Define success criteria for subtask"""
        criteria_map = {
            'navigate_to': 'Robot reaches target location within tolerance',
            'detect_object': 'Object detected with confidence > 0.8',
            'grasp_object': 'Object successfully grasped and verified',
            'place_object': 'Object successfully placed at target location',
            'greet_person': 'Person acknowledged greeting',
            'detect_person': 'Person detected in front of robot'
        }

        action_key = subtask_template['action'].replace('_', '')
        return criteria_map.get(action_key, 'Action completed successfully')

    def _generate_dynamic_plan(self, command: str) -> Optional[TaskPlan]:
        """Generate plan for unknown task types using LLM"""
        # This would call the LLM planner to generate a custom plan
        # For now, return a simple plan as placeholder
        steps = [
            TaskStep(
                id=0,
                type='communication',
                action='acknowledge_command',
                parameters={'command': command},
                description='Acknowledge received command',
                priority=TaskPriority.HIGH,
                dependencies=[],
                estimated_duration=1.0,
                success_criteria='Command acknowledged'
            )
        ]

        plan = TaskPlan(
            id=f"dynamic_{self.get_clock().now().nanoseconds}",
            task_description=f"Dynamic task: {command}",
            steps=steps,
            estimated_duration=1.0,
            confidence=0.7,  # Lower confidence for dynamic tasks
            created_at=self.get_clock().now().nanoseconds / 1e9
        )

        return plan

    def _publish_task_plan(self, plan: TaskPlan):
        """Publish task plan to execution system"""
        plan_dict = {
            'id': plan.id,
            'task_description': plan.task_description,
            'steps': [
                {
                    'id': step.id,
                    'type': step.type,
                    'action': step.action,
                    'parameters': step.parameters,
                    'description': step.description,
                    'priority': step.priority.value,
                    'dependencies': step.dependencies,
                    'estimated_duration': step.estimated_duration,
                    'success_criteria': step.success_criteria
                }
                for step in plan.steps
            ],
            'estimated_duration': plan.estimated_duration,
            'confidence': plan.confidence,
            'created_at': plan.created_at
        }

        plan_msg = String()
        plan_msg.data = json.dumps(plan_dict)
        self.task_plan_pub.publish(plan_msg)

        self.get_logger().info(f'Published task plan with {len(plan.steps)} steps')

def main(args=None):
    rclpy.init(args=args)
    node = HierarchicalPlannerNode()

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

## Reactive Planning and Adaptation

### Context-Aware Planning

```python
# reactive_planning.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
import json
import asyncio
from typing import Dict, List, Optional, Any
from dataclasses import dataclass

@dataclass
class EnvironmentalContext:
    detected_objects: List[Dict]
    robot_state: Dict
    environment_map: Dict
    obstacles: List[Dict]
    current_task: Optional[str]
    timestamp: float

class ReactivePlannerNode(Node):
    def __init__(self):
        super().__init__('reactive_planner')

        # Initialize reactive planning components
        self._initialize_reactive_components()

        # Subscribers for context information
        self.perception_sub = self.create_subscription(
            String,
            '/perception_context',
            self.perception_callback,
            10
        )

        self.robot_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.robot_state_callback,
            10
        )

        self.map_sub = self.create_subscription(
            String,
            '/environment_map',
            self.map_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            String,
            '/obstacles',
            self.obstacle_callback,
            10
        )

        # Publishers
        self.adapted_plan_pub = self.create_publisher(String, '/adapted_plan', 10)
        self.replan_request_pub = self.create_publisher(Bool, '/replan_request', 10)

        # Context management
        self.current_context = EnvironmentalContext(
            detected_objects=[],
            robot_state={},
            environment_map={},
            obstacles=[],
            current_task=None,
            timestamp=0.0
        )

        self.active_plan = None
        self.context_change_threshold = 0.1  # Significant change threshold

        self.get_logger().info('Reactive Planner initialized')

    def _initialize_reactive_components(self):
        """Initialize reactive planning components"""
        # Context change detection parameters
        self.previous_context_signature = ""
        self.context_change_detection = True

        # Plan adaptation rules
        self.adaptation_rules = [
            {
                'condition': 'new_obstacle_detected',
                'action': 'reroute_navigation',
                'priority': 'high'
            },
            {
                'condition': 'object_moved',
                'action': 'update_target_location',
                'priority': 'medium'
            },
            {
                'condition': 'robot_low_battery',
                'action': 'return_to_charging',
                'priority': 'critical'
            },
            {
                'condition': 'grasp_failed',
                'action': 'retry_grasp_or_abort',
                'priority': 'high'
            }
        ]

        self.get_logger().info('Reactive planning components initialized')

    def perception_callback(self, msg):
        """Update context with perception data"""
        try:
            data = json.loads(msg.data)
            self.current_context.detected_objects = data.get('objects', [])
            self.current_context.timestamp = self.get_clock().now().nanoseconds / 1e9

            # Check for context changes that require plan adaptation
            self._check_context_changes()

        except Exception as e:
            self.get_logger().error(f'Perception context update error: {e}')

    def robot_state_callback(self, msg):
        """Update context with robot state"""
        try:
            robot_state = {
                'joint_positions': dict(zip(msg.name, msg.position)),
                'joint_velocities': dict(zip(msg.name, msg.velocity)),
                'joint_efforts': dict(zip(msg.name, msg.effort)),
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            }

            self.current_context.robot_state = robot_state
            self.current_context.timestamp = self.current_context.robot_state['timestamp']

            # Check for state changes that require plan adaptation
            self._check_context_changes()

        except Exception as e:
            self.get_logger().error(f'Robot state update error: {e}')

    def map_callback(self, msg):
        """Update context with environment map"""
        try:
            self.current_context.environment_map = json.loads(msg.data)
            self.current_context.timestamp = self.get_clock().now().nanoseconds / 1e9

            self._check_context_changes()

        except Exception as e:
            self.get_logger().error(f'Map update error: {e}')

    def obstacle_callback(self, msg):
        """Update context with obstacle information"""
        try:
            self.current_context.obstacles = json.loads(msg.data).get('obstacles', [])
            self.current_context.timestamp = self.get_clock().now().nanoseconds / 1e9

            # Check for new obstacles that require plan adaptation
            self._check_context_changes()

        except Exception as e:
            self.get_logger().error(f'Obstacle update error: {e}')

    def _check_context_changes(self):
        """Check if context changes require plan adaptation"""
        try:
            # Create context signature for change detection
            context_signature = self._create_context_signature()

            if context_signature != self.previous_context_signature:
                # Context has changed significantly
                self.get_logger().info('Context change detected, evaluating plan adaptation')

                # Determine appropriate adaptation
                adaptation_needed = self._evaluate_adaptation_needed()

                if adaptation_needed:
                    self._trigger_plan_adaptation(adaptation_needed)

                self.previous_context_signature = context_signature

        except Exception as e:
            self.get_logger().error(f'Context change evaluation error: {e}')

    def _create_context_signature(self) -> str:
        """Create signature of current context for change detection"""
        import hashlib

        # Create hash of important context elements
        context_elements = {
            'object_count': len(self.current_context.detected_objects),
            'obstacle_count': len(self.current_context.obstacles),
            'robot_pose': self.current_context.robot_state.get('pose', {}),
            'timestamp': int(self.current_context.timestamp * 100)  # Round to 10ms
        }

        signature_data = json.dumps(context_elements, sort_keys=True)
        return hashlib.md5(signature_data.encode()).hexdigest()

    def _evaluate_adaptation_needed(self) -> Optional[Dict[str, Any]]:
        """Evaluate if plan adaptation is needed"""
        adaptations = []

        # Check for new obstacles blocking current path
        if self.active_plan and self.current_context.obstacles:
            for obstacle in self.current_context.obstacles:
                if self._obstacle_blocks_current_path(obstacle):
                    adaptations.append({
                        'type': 'path_obstruction',
                        'obstacle': obstacle,
                        'action': 'find_alternative_path'
                    })

        # Check for moved target objects
        if self.active_plan and self.current_context.detected_objects:
            for obj in self.current_context.detected_objects:
                if self._object_is_target_being_sought(obj):
                    # Check if object position has changed significantly
                    if self._significant_position_change(obj):
                        adaptations.append({
                            'type': 'target_moved',
                            'object': obj,
                            'action': 'update_target_location'
                        })

        # Check for robot state changes
        if self.current_context.robot_state:
            battery_level = self.current_context.robot_state.get('battery_level', 100.0)
            if battery_level < 20.0:  # Low battery threshold
                adaptations.append({
                    'type': 'low_battery',
                    'battery_level': battery_level,
                    'action': 'return_to_charging'
                })

        return adaptations[0] if adaptations else None

    def _obstacle_blocks_current_path(self, obstacle: Dict) -> bool:
        """Check if obstacle blocks current navigation path"""
        if not self.active_plan or not self.active_plan.get('current_path'):
            return False

        # Check if obstacle is on or near current path
        # This would involve path geometry analysis
        obstacle_pos = obstacle.get('position', {})
        path_points = self.active_plan.get('current_path', [])

        for point in path_points[:10]:  # Check first 10 points of path
            if self._distance_3d(obstacle_pos, point) < 0.5:  # Within 50cm
                return True

        return False

    def _object_is_target_being_sought(self, obj: Dict) -> bool:
        """Check if object is a current target"""
        if not self.active_plan or not self.active_plan.get('current_target'):
            return False

        target_desc = self.active_plan['current_target']
        return obj.get('class', '').lower() in target_desc.lower()

    def _significant_position_change(self, obj: Dict) -> bool:
        """Check if object position has changed significantly"""
        if not self.active_plan or not self.active_plan.get('original_target_position'):
            return False

        original_pos = self.active_plan['original_target_position']
        current_pos = obj.get('position', {})

        distance = self._distance_3d(original_pos, current_pos)
        return distance > 0.3  # 30cm threshold

    def _trigger_plan_adaptation(self, adaptation_info: Dict[str, Any]):
        """Trigger plan adaptation based on context change"""
        self.get_logger().info(f'Adapting plan due to: {adaptation_info["type"]}')

        if adaptation_info['action'] == 'find_alternative_path':
            self._adapt_navigation_path(adaptation_info['obstacle'])
        elif adaptation_info['action'] == 'update_target_location':
            self._update_target_location(adaptation_info['object'])
        elif adaptation_info['action'] == 'return_to_charging':
            self._adapt_for_low_battery(adaptation_info['battery_level'])

    def _adapt_navigation_path(self, obstacle: Dict):
        """Adapt navigation plan to avoid obstacle"""
        # Request replanning with obstacle avoidance
        replan_msg = Bool()
        replan_msg.data = True
        self.replan_request_pub.publish(replan_msg)

        self.get_logger().info(f'Navigation path adapted to avoid obstacle at {obstacle.get("position")}')

    def _update_target_location(self, obj: Dict):
        """Update plan with new target location"""
        if self.active_plan:
            self.active_plan['current_target_position'] = obj.get('position', {})
            self.get_logger().info(f'Target location updated to {obj.get("position")}')

    def _adapt_for_low_battery(self, battery_level: float):
        """Adapt plan for low battery situation"""
        # Insert charging station navigation into current plan
        charging_plan = {
            'task': f'Battery level {battery_level}%, returning to charging station',
            'steps': [{
                'id': 999,  # High priority step
                'type': 'navigation',
                'action': 'navigate_to',
                'parameters': {'location': 'charging_station'},
                'description': f'Emergency charging due to low battery ({battery_level}%)',
                'priority': 1,
                'dependencies': []
            }],
            'estimated_duration': 120.0,  # 2 minutes to charging station
            'confidence': 1.0,
            'created_at': self.get_clock().now().nanoseconds / 1e9
        }

        # Publish emergency charging plan
        plan_msg = String()
        plan_msg.data = json.dumps(charging_plan)
        self.adapted_plan_pub.publish(plan_msg)

        self.get_logger().warn(f'Emergency charging triggered: battery level {battery_level}%')

    def _distance_3d(self, pos1: Dict, pos2: Dict) -> float:
        """Calculate 3D Euclidean distance between positions"""
        try:
            x1 = pos1.get('x', 0)
            y1 = pos1.get('y', 0)
            z1 = pos1.get('z', 0)

            x2 = pos2.get('x', 0)
            y2 = pos2.get('y', 0)
            z2 = pos2.get('z', 0)

            return ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5
        except:
            return float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = ReactivePlannerNode()

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

## LLM-Based Reasoning and Planning

### Advanced Reasoning Capabilities

```python
# advanced_reasoning.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import json
import asyncio
from typing import Dict, List, Optional, Any
import openai
import numpy as np

class AdvancedReasoningNode(Node):
    def __init__(self):
        super().__init__('advanced_reasoning')

        # Initialize advanced reasoning components
        self._initialize_reasoning_components()

        # Subscribers
        self.reasoning_request_sub = self.create_subscription(
            String,
            '/reasoning_request',
            self.reasoning_request_callback,
            10
        )

        self.context_update_sub = self.create_subscription(
            String,
            '/context_update',
            self.context_update_callback,
            10
        )

        # Publishers
        self.reasoning_result_pub = self.create_publisher(String, '/reasoning_result', 10)
        self.explanation_pub = self.create_publisher(String, '/explanation', 10)

        # Reasoning state
        self.current_context = {}
        self.reasoning_history = []
        self.belief_state = {}

        self.get_logger().info('Advanced Reasoning node initialized')

    def _initialize_reasoning_components(self):
        """Initialize advanced reasoning components"""
        try:
            # Initialize LLM for reasoning
            api_key = self.declare_parameter('openai_api_key', '').value
            if api_key:
                self.llm_client = openai.OpenAI(api_key=api_key)
                self.reasoning_model = self.declare_parameter('reasoning_model', 'gpt-4-turbo').value
                self.get_logger().info('LLM reasoning initialized')
            else:
                self.llm_client = None
                self.get_logger().warn('No LLM API key provided, using rule-based reasoning')

            # Initialize belief tracking
            self._initialize_belief_tracking()

            # Initialize causal reasoning
            self._initialize_causal_reasoning()

        except Exception as e:
            self.get_logger().error(f'Reasoning initialization error: {e}')
            raise

    def _initialize_belief_tracking(self):
        """Initialize belief tracking system"""
        self.belief_state = {
            'objects': {},  # Object states and locations
            'locations': {},  # Location states
            'events': [],  # Recent events
            'goals': [],  # Active goals
            'intentions': [],  # Robot intentions
            'knowledge': {}  # General knowledge
        }

    def _initialize_causal_reasoning(self):
        """Initialize causal reasoning system"""
        self.causal_rules = {
            'object_manipulation': [
                {
                    'if': 'robot_grasps_object(X)',
                    'then': 'robot_has_object(X) AND object_not_at_original_location(X)'
                },
                {
                    'if': 'robot_places_object(X, location)',
                    'then': 'object_at_location(X, location) AND robot_not_holding_object(X)'
                }
            ],
            'navigation': [
                {
                    'if': 'robot_navigates_to(location)',
                    'then': 'robot_at_location(location)'
                },
                {
                    'if': 'obstacle_appears_at_path()',
                    'then': 'path_blocked() AND need_new_route()'
                }
            ],
            'perception': [
                {
                    'if': 'object_detected(X)',
                    'then': 'robot_knows_object_location(X)'
                },
                {
                    'if': 'object_moved(X)',
                    'then': 'previous_location_invalid(X)'
                }
            ]
        }

    def reasoning_request_callback(self, msg):
        """Process reasoning request"""
        try:
            request_data = json.loads(msg.data)
            query = request_data.get('query', '')
            context = request_data.get('context', {})

            # Perform reasoning
            result = self._perform_reasoning(query, context)

            if result:
                # Publish reasoning result
                result_msg = String()
                result_msg.data = json.dumps(result)
                self.reasoning_result_pub.publish(result_msg)

                # Publish explanation
                explanation_msg = String()
                explanation_msg.data = result.get('explanation', 'No explanation provided')
                self.explanation_pub.publish(explanation_msg)

                self.get_logger().info(f'Reasoning completed: {query}')

        except Exception as e:
            self.get_logger().error(f'Reasoning request error: {e}')

    def _perform_reasoning(self, query: str, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Perform advanced reasoning based on query and context"""
        try:
            # Update context
            self.current_context.update(context)

            # Determine reasoning type
            reasoning_type = self._determine_reasoning_type(query)

            if reasoning_type == 'causal':
                result = self._perform_causal_reasoning(query, context)
            elif reasoning_type == 'planning':
                result = self._perform_planning_reasoning(query, context)
            elif reasoning_type == 'explanation':
                result = self._perform_explanation_reasoning(query, context)
            elif reasoning_type == 'prediction':
                result = self._perform_predictive_reasoning(query, context)
            else:
                result = self._perform_general_reasoning(query, context)

            # Update belief state
            self._update_belief_state(result)

            return result

        except Exception as e:
            self.get_logger().error(f'Reasoning execution error: {e}')
            return None

    def _determine_reasoning_type(self, query: str) -> str:
        """Determine the type of reasoning required"""
        query_lower = query.lower()

        causal_keywords = ['why', 'because', 'cause', 'effect', 'due to', 'led to']
        planning_keywords = ['how to', 'plan', 'strategy', 'approach', 'method', 'procedure']
        explanation_keywords = ['explain', 'describe', 'what happened', 'tell me about']
        prediction_keywords = ['will', 'going to', 'predict', 'expect', 'future', 'next']

        if any(keyword in query_lower for keyword in causal_keywords):
            return 'causal'
        elif any(keyword in query_lower for keyword in planning_keywords):
            return 'planning'
        elif any(keyword in query_lower for keyword in explanation_keywords):
            return 'explanation'
        elif any(keyword in query_lower for keyword in prediction_keywords):
            return 'prediction'
        else:
            return 'general'

    def _perform_causal_reasoning(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Perform causal reasoning"""
        try:
            # Use LLM for complex causal reasoning
            if self.llm_client:
                return self._llm_causal_reasoning(query, context)
            else:
                # Rule-based causal reasoning
                return self._rule_based_causal_reasoning(query, context)

        except Exception as e:
            self.get_logger().error(f'Causal reasoning error: {e}')
            return self._fallback_reasoning(query, context)

    def _llm_causal_reasoning(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Perform LLM-based causal reasoning"""
        system_prompt = f"""
        You are an AI reasoning assistant for a humanoid robot. Your role is to analyze causal relationships in the robot's environment and actions.

        Context: {json.dumps(context, indent=2)}

        Belief State: {json.dumps(self.belief_state, indent=2)}

        Please analyze the causal relationships in the query and provide:
        1. The cause-and-effect chain
        2. Contributing factors
        3. Potential consequences
        4. Explanation of reasoning process
        """

        user_prompt = f"Query: {query}\n\nAnalyze the causal relationships and provide a detailed explanation."

        try:
            response = self.llm_client.chat.completions.create(
                model=self.reasoning_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            result = json.loads(response.choices[0].message.content)

            return {
                'type': 'causal_reasoning',
                'result': result,
                'explanation': result.get('explanation', ''),
                'confidence': result.get('confidence', 0.8),
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            }

        except Exception as e:
            self.get_logger().error(f'LLM causal reasoning error: {e}')
            return self._rule_based_causal_reasoning(query, context)

    def _rule_based_causal_reasoning(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Perform rule-based causal reasoning"""
        # Apply causal rules to determine cause-and-effect
        causal_chain = []
        contributing_factors = []
        potential_consequences = []

        # Analyze query for causal patterns
        for rule_set in self.causal_rules.values():
            for rule in rule_set:
                # This is a simplified rule matching
                # In practice, this would use more sophisticated pattern matching
                if any(keyword in query.lower() for keyword in ['grasp', 'pick', 'take']):
                    causal_chain.append('Robot attempted to grasp object')
                    potential_consequences.append('Object may be in robot\'s gripper')
                    break

        return {
            'type': 'causal_reasoning',
            'result': {
                'causal_chain': causal_chain,
                'contributing_factors': contributing_factors,
                'potential_consequences': potential_consequences
            },
            'explanation': f'Rule-based causal analysis for: {query}',
            'confidence': 0.7,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

    def _perform_predictive_reasoning(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Perform predictive reasoning about future states"""
        try:
            # Use LLM for complex predictions
            if self.llm_client:
                return self._llm_predictive_reasoning(query, context)
            else:
                # Simple prediction based on current state
                return self._simple_predictive_reasoning(query, context)

        except Exception as e:
            self.get_logger().error(f'Predictive reasoning error: {e}')
            return self._fallback_reasoning(query, context)

    def _llm_predictive_reasoning(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Perform LLM-based predictive reasoning"""
        system_prompt = f"""
        You are an AI prediction assistant for a humanoid robot. Predict future states, events, or outcomes based on the current context.

        Current Context: {json.dumps(context, indent=2)}
        Belief State: {json.dumps(self.belief_state, indent=2)}

        Predict the most likely future outcomes considering:
        1. Robot's current state and goals
        2. Environmental conditions
        3. Past experiences and learned patterns
        4. Physical constraints and possibilities
        """

        user_prompt = f"Query: {query}\n\nPredict the most likely future outcome(s) and explain your reasoning."

        try:
            response = self.llm_client.chat.completions.create(
                model=self.reasoning_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                response_format={"type": "json_object"}
            )

            prediction = json.loads(response.choices[0].message.content)

            return {
                'type': 'predictive_reasoning',
                'result': prediction,
                'explanation': prediction.get('reasoning', ''),
                'confidence': prediction.get('confidence', 0.7),
                'predictions': prediction.get('predictions', []),
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            }

        except Exception as e:
            self.get_logger().error(f'LLM predictive reasoning error: {e}')
            return self._simple_predictive_reasoning(query, context)

    def _update_belief_state(self, reasoning_result: Dict[str, Any]):
        """Update belief state based on reasoning result"""
        try:
            # Update based on reasoning type
            result_type = reasoning_result.get('type', '')
            result_data = reasoning_result.get('result', {})

            if result_type == 'causal_reasoning':
                # Update causal beliefs
                pass
            elif result_type == 'predictive_reasoning':
                # Update predictions
                predictions = result_data.get('predictions', [])
                for pred in predictions:
                    # Add to belief state
                    pass

            # Add reasoning result to history
            self.reasoning_history.append(reasoning_result)
            if len(self.reasoning_history) > 100:  # Keep last 100 reasoning results
                self.reasoning_history.pop(0)

        except Exception as e:
            self.get_logger().error(f'Belief state update error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedReasoningNode()

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

### Efficient LLM Integration

```python
# performance_optimization.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
import time
from typing import Dict, Any, Optional

class OptimizedLLMNode(Node):
    def __init__(self):
        super().__init__('optimized_llm_planner')

        # Initialize optimized components
        self._initialize_optimized_components()

        # Initialize thread pool for LLM calls
        self.executor = ThreadPoolExecutor(max_workers=2)

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            '/optimized_command',
            self.optimized_command_callback,
            10
        )

        self.performance_pub = self.create_publisher(Float32, '/llm_performance', 10)

        # Performance tracking
        self.request_times = []
        self.response_times = []

        self.get_logger().info('Optimized LLM Planner initialized')

    def _initialize_optimized_components(self):
        """Initialize optimized LLM components"""
        try:
            # Initialize LLM with optimizations
            api_key = self.declare_parameter('openai_api_key', '').value
            if api_key:
                self.llm_client = openai.AsyncOpenAI(api_key=api_key)
                self.model_name = self.declare_parameter('model_name', 'gpt-4-turbo').value

                # Initialize caching
                self.response_cache = {}
                self.cache_size_limit = 100

                # Initialize rate limiting
                self.request_times = []
                self.max_requests_per_minute = 3000  # Adjust based on your API tier

                self.get_logger().info('Optimized LLM components initialized')

            else:
                self.get_logger().error('No API key provided for optimized LLM')
                raise ValueError('No API key provided')

        except Exception as e:
            self.get_logger().error(f'Optimized LLM initialization error: {e}')
            raise

    def optimized_command_callback(self, msg):
        """Process command with optimized LLM integration"""
        command_text = msg.data

        # Submit to thread pool for non-blocking execution
        future = self.executor.submit(self._process_command_optimized, command_text)
        future.add_done_callback(self._handle_completion)

    def _process_command_optimized(self, command: str) -> Optional[Dict[str, Any]]:
        """Process command with optimizations"""
        start_time = time.time()

        try:
            # Check cache first
            cached_result = self._check_cache(command)
            if cached_result:
                self.get_logger().info('Cache hit for command')
                return cached_result

            # Check rate limiting
            if not self._check_rate_limit():
                self.get_logger().warn('Rate limit exceeded, queuing request')
                # Implement queuing mechanism
                time.sleep(0.1)  # Brief delay before retry

            # Prepare optimized prompt
            optimized_prompt = self._prepare_optimized_prompt(command)

            # Make optimized API call
            response = asyncio.run(self._make_optimized_call(optimized_prompt))

            if response:
                # Cache result
                self._cache_result(command, response)

                # Calculate and log performance
                processing_time = time.time() - start_time
                self.response_times.append(processing_time)

                self.get_logger().info(f'Optimized processing time: {processing_time:.3f}s')

                return response

        except Exception as e:
            self.get_logger().error(f'Optimized processing error: {e}')
            return None

    async def _make_optimized_call(self, prompt: str) -> Optional[Dict[str, Any]]:
        """Make optimized LLM API call"""
        try:
            response = await self.llm_client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Lower for more consistent results
                max_tokens=500,
                timeout=30.0  # 30 second timeout
            )

            return response.choices[0].message.content

        except Exception as e:
            self.get_logger().error(f'LLM API call error: {e}')
            return None

    def _prepare_optimized_prompt(self, command: str) -> str:
        """Prepare optimized prompt for LLM"""
        # Use shorter, more focused prompts
        # Include only relevant context
        # Use structured format for better parsing

        return f"""
        Command: {command}

        Respond with a structured JSON plan containing:
        {{
            "action": "action_to_perform",
            "parameters": {{"param1": "value1"}},
            "confidence": 0.0-1.0
        }}

        Be concise and specific.
        """

    def _check_cache(self, command: str) -> Optional[Dict[str, Any]]:
        """Check cache for previously processed command"""
        import hashlib
        command_hash = hashlib.md5(command.encode()).hexdigest()

        if command_hash in self.response_cache:
            return self.response_cache[command_hash]

        return None

    def _cache_result(self, command: str, result: Dict[str, Any]):
        """Cache result with size management"""
        import hashlib
        command_hash = hashlib.md5(command.encode()).hexdigest()

        # Remove oldest entries if cache is full
        if len(self.response_cache) >= self.cache_size_limit:
            # Remove first item (oldest)
            oldest_key = next(iter(self.response_cache))
            del self.response_cache[oldest_key]

        self.response_cache[command_hash] = result

    def _check_rate_limit(self) -> bool:
        """Check if we're within API rate limits"""
        current_time = time.time()

        # Remove requests older than 1 minute
        self.request_times = [req_time for req_time in self.request_times
                             if current_time - req_time < 60]

        # Check if we're under the limit
        if len(self.request_times) < self.max_requests_per_minute:
            self.request_times.append(current_time)
            return True

        return False

    def _handle_completion(self, future):
        """Handle completion of async task"""
        try:
            result = future.result()
            if result:
                # Publish result
                result_msg = String()
                result_msg.data = json.dumps(result)
                self.result_pub.publish(result_msg)

                # Publish performance metric
                if self.response_times:
                    avg_time = sum(self.response_times[-10:]) / len(self.response_times[-10:])
                    perf_msg = Float32()
                    perf_msg.data = avg_time
                    self.performance_pub.publish(perf_msg)

        except Exception as e:
            self.get_logger().error(f'Task completion error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedLLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.executor.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### LLM Integration Best Practices

1. **Prompt Engineering**: Craft precise, structured prompts for consistent results
2. **Caching**: Cache frequent queries to reduce API calls and improve response time
3. **Rate Limiting**: Respect API rate limits to avoid service disruptions
4. **Error Handling**: Implement robust error handling and fallback mechanisms
5. **Security**: Secure API keys and sensitive data appropriately
6. **Validation**: Validate LLM outputs before using for robot control
7. **Monitoring**: Monitor LLM performance and costs continuously
8. **Context Management**: Provide relevant context while minimizing token usage
9. **Confidence Scoring**: Use confidence scores to determine reliability
10. **Privacy**: Consider privacy implications of sending data to cloud services

### Performance Optimization Tips

1. **Async Processing**: Use asynchronous calls to avoid blocking
2. **Batch Requests**: Combine multiple requests when possible
3. **Response Streaming**: Use streaming for long responses
4. **Model Selection**: Choose appropriate models for your needs
5. **Token Management**: Optimize prompt length for cost and performance
6. **Caching Strategies**: Implement smart caching for repeated queries
7. **Fallback Mechanisms**: Have local alternatives for critical functions
8. **Load Balancing**: Distribute requests across multiple models/providers

## Troubleshooting

### Common Issues and Solutions

**Issue**: High API costs
**Solution**: Implement caching, optimize prompt length, use cheaper models for simple tasks

**Issue**: Slow response times
**Solution**: Use async processing, implement local fallbacks, optimize prompts

**Issue**: Inconsistent outputs
**Solution**: Improve prompt engineering, add response validation, use lower temperatures

**Issue**: Rate limiting
**Solution**: Implement proper rate limiting, use request queuing, consider enterprise plans

**Issue**: Security concerns
**Solution**: Use secure key management, implement data sanitization, consider on-premise options

## Next Steps

In the next chapter, we'll explore the Safety and Guardrails system that ensures our AI-driven humanoid robot operates safely and reliably. We'll learn how to implement comprehensive safety checks, validation systems, and fail-safe mechanisms that protect both the robot and its environment while maintaining the advanced capabilities we've developed.