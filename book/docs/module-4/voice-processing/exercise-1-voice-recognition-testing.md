# Exercise 1: Voice Recognition Testing with Different Commands

## Objective
Test voice recognition capabilities with various types of commands and evaluate recognition accuracy across different scenarios and environments.

## Prerequisites
- Completed setup tutorial
- Working voice processing system
- Microphone hardware
- OpenAI API key configured
- Vosk model installed

## Exercise Steps

### Step 1: Create Test Command Categories

Create a file `test_commands.py`:

```python
#!/usr/bin/env python3
"""
Test commands for voice recognition evaluation
"""
import json
import csv
from typing import List, Dict, Tuple

class VoiceCommandTestSuite:
    """Test suite for voice recognition with different command types"""

    def __init__(self):
        self.test_commands = {
            'basic_movement': [
                ("move forward", "move", {"direction": "forward"}),
                ("move backward", "move", {"direction": "backward"}),
                ("move left", "move", {"direction": "left"}),
                ("move right", "move", {"direction": "right"}),
                ("go forward", "move", {"direction": "forward"}),
                ("go backward", "move", {"direction": "backward"}),
                ("walk forward", "move", {"direction": "forward"}),
                ("walk backward", "move", {"direction": "backward"})
            ],
            'movement_with_distance': [
                ("move forward 2 meters", "move", {"direction": "forward", "distance": 2.0, "unit": "meters"}),
                ("move left 1.5 meters", "move", {"direction": "left", "distance": 1.5, "unit": "meters"}),
                ("go forward 3 meters", "move", {"direction": "forward", "distance": 3.0, "unit": "meters"}),
                ("walk right 0.5 meters", "move", {"direction": "right", "distance": 0.5, "unit": "meters"})
            ],
            'navigation': [
                ("go to kitchen", "navigate", {"location": "kitchen"}),
                ("go to bedroom", "navigate", {"location": "bedroom"}),
                ("go to office", "navigate", {"location": "office"}),
                ("go to living room", "navigate", {"location": "living_room"}),
                ("navigate to kitchen", "navigate", {"location": "kitchen"}),
                ("move to bedroom", "navigate", {"location": "bedroom"}),
                ("head to office", "navigate", {"location": "office"})
            ],
            'grasping': [
                ("grasp the ball", "grasp", {"object": "ball"}),
                ("pick up the red ball", "grasp", {"object": "red ball"}),
                ("grab the cup", "grasp", {"object": "cup"}),
                ("take the book", "grasp", {"object": "book"}),
                ("pick the book up", "grasp", {"object": "book"})
            ],
            'inspection': [
                ("look at the ball", "inspect", {"object": "ball"}),
                ("inspect the red ball", "inspect", {"object": "red ball"}),
                ("check the cup", "inspect", {"object": "cup"}),
                ("observe the book", "inspect", {"object": "book"}),
                ("find the red ball", "inspect", {"object": "red ball"})
            ],
            'stop_commands': [
                ("stop", "stop", {}),
                ("halt", "stop", {}),
                ("pause", "stop", {}),
                ("freeze", "stop", {})
            ],
            'complex_commands': [
                ("bring the red ball to kitchen", "bring", {"object": "red ball", "location": "kitchen"}),
                ("take the cup to bedroom", "bring", {"object": "cup", "location": "bedroom"}),
                ("carry the book to office", "bring", {"object": "book", "location": "office"})
            ]
        }

        # Add variations for testing robustness
        self.variation_commands = [
            ("move forwards", "move", {"direction": "forward"}),
            ("go backwards", "move", {"direction": "backward"}),
            ("go 2 meters forward", "move", {"distance": 2.0, "direction": "forward", "unit": "meters"}),
            ("head to living room", "navigate", {"location": "living_room"}),
            ("pick up the blue cup", "grasp", {"object": "blue cup"}),
            ("check the green book", "inspect", {"object": "green book"})
        ]

    def get_all_commands(self) -> List[Tuple[str, str, Dict]]:
        """Get all test commands from all categories"""
        all_commands = []
        for category, commands in self.test_commands.items():
            all_commands.extend(commands)
        all_commands.extend(self.variation_commands)
        return all_commands

    def get_commands_by_category(self, category: str) -> List[Tuple[str, str, Dict]]:
        """Get commands for a specific category"""
        if category in self.test_commands:
            return self.test_commands[category]
        elif category == 'variations':
            return self.variation_commands
        else:
            raise ValueError(f"Unknown category: {category}")

    def save_commands_to_file(self, filename: str):
        """Save commands to a CSV file for external testing"""
        all_commands = self.get_all_commands()

        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['command_text', 'expected_intent', 'expected_entities'])

            for command_text, expected_intent, expected_entities in all_commands:
                writer.writerow([command_text, expected_intent, json.dumps(expected_entities)])

    def load_commands_from_file(self, filename: str) -> List[Tuple[str, str, Dict]]:
        """Load commands from a CSV file"""
        commands = []

        with open(filename, 'r', encoding='utf-8') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip header

            for row in reader:
                command_text = row[0]
                expected_intent = row[1]
                expected_entities = json.loads(row[2])
                commands.append((command_text, expected_intent, expected_entities))

        return commands

class RecognitionTestRunner:
    """Run tests and evaluate recognition performance"""

    def __init__(self, command_parser):
        self.command_parser = command_parser
        self.test_suite = VoiceCommandTestSuite()

    def run_test(self, command_text: str, expected_intent: str, expected_entities: Dict) -> Dict:
        """Run a single test case"""
        result = {
            'command_text': command_text,
            'expected_intent': expected_intent,
            'expected_entities': expected_entities,
            'parsed_intent': None,
            'parsed_entities': {},
            'confidence': 0.0,
            'intent_correct': False,
            'entities_correct': False,
            'overall_correct': False
        }

        # Parse the command
        parsed_command = self.command_parser.parse_command(command_text)

        if parsed_command:
            result['parsed_intent'] = parsed_command.intent
            result['parsed_entities'] = parsed_command.entities
            result['confidence'] = parsed_command.confidence

            # Check if intent is correct
            result['intent_correct'] = parsed_command.intent == expected_intent

            # Check if entities are correct (simplified comparison)
            result['entities_correct'] = self._compare_entities(
                parsed_command.entities, expected_entities
            )

            # Overall correctness
            result['overall_correct'] = result['intent_correct'] and result['entities_correct']

        return result

    def _compare_entities(self, parsed_entities: Dict, expected_entities: Dict) -> bool:
        """Compare parsed entities with expected entities"""
        if not expected_entities:  # If no expected entities, any parsed entities are acceptable
            return True

        for key, expected_value in expected_entities.items():
            if key not in parsed_entities:
                return False

            parsed_value = parsed_entities[key]

            # Handle numeric comparison
            if isinstance(expected_value, (int, float)):
                if not isinstance(parsed_value, (int, float)):
                    return False
                if abs(parsed_value - expected_value) > 0.1:  # Allow small floating point differences
                    return False
            else:
                # Convert to string for comparison (handle different formats)
                if str(parsed_value).lower() != str(expected_value).lower():
                    return False

        return True

    def run_all_tests(self) -> List[Dict]:
        """Run all test commands and return results"""
        all_commands = self.test_suite.get_all_commands()
        results = []

        for command_text, expected_intent, expected_entities in all_commands:
            result = self.run_test(command_text, expected_intent, expected_entities)
            results.append(result)

        return results

    def run_category_tests(self, category: str) -> List[Dict]:
        """Run tests for a specific category"""
        commands = self.test_suite.get_commands_by_category(category)
        results = []

        for command_text, expected_intent, expected_entities in commands:
            result = self.run_test(command_text, expected_intent, expected_entities)
            results.append(result)

        return results

    def generate_report(self, results: List[Dict]) -> Dict:
        """Generate a performance report from test results"""
        total_tests = len(results)
        correct_intent_tests = sum(1 for r in results if r['intent_correct'])
        correct_entity_tests = sum(1 for r in results if r['entities_correct'])
        correct_overall_tests = sum(1 for r in results if r['overall_correct'])

        avg_confidence = sum(r['confidence'] for r in results if r['confidence'] > 0) / len([r for r in results if r['confidence'] > 0]) if any(r['confidence'] > 0 for r in results) else 0

        report = {
            'total_tests': total_tests,
            'correct_intent_tests': correct_intent_tests,
            'correct_entity_tests': correct_entity_tests,
            'correct_overall_tests': correct_overall_tests,
            'intent_accuracy': correct_intent_tests / total_tests if total_tests > 0 else 0,
            'entity_accuracy': correct_entity_tests / total_tests if total_tests > 0 else 0,
            'overall_accuracy': correct_overall_tests / total_tests if total_tests > 0 else 0,
            'average_confidence': avg_confidence,
            'detailed_results': results
        }

        return report

# Example usage
if __name__ == "__main__":
    # This would be used with your command parser
    # from command_parser import VoiceCommandParser

    # parser = VoiceCommandParser()
    # runner = RecognitionTestRunner(parser)
    # results = runner.run_all_tests()
    # report = runner.generate_report(results)
    # print(f"Overall accuracy: {report['overall_accuracy']:.2%}")

    # Create the test suite
    test_suite = VoiceCommandTestSuite()
    test_suite.save_commands_to_file("voice_test_commands.csv")
    print("Test commands saved to voice_test_commands.csv")
```

### Step 2: Create Recognition Testing Script

Create a file `test_recognition.py`:

```python
#!/usr/bin/env python3
"""
Voice recognition testing script
Tests the system with various commands and evaluates performance
"""
import sys
import time
import os
import numpy as np
import wave
from typing import List, Dict, Tuple
from test_commands import VoiceCommandTestSuite, RecognitionTestRunner

def simulate_speech_recognition(text: str) -> str:
    """
    Simulate speech recognition for testing purposes
    In a real system, this would call the actual recognition service
    """
    # This is a placeholder - in real implementation, this would call Whisper or Vosk
    print(f"Simulating recognition for: '{text}'")
    return text  # Return the same text for simulation

def record_and_test_command(command_text: str, duration: float = 3.0) -> str:
    """
    Record audio for a command and return transcribed text
    This is a placeholder implementation
    """
    print(f"Recording for command: '{command_text}'")
    # In a real implementation, this would:
    # 1. Record audio for the specified duration
    # 2. Process the audio through the recognition system
    # 3. Return the transcribed text

    # For simulation, return the command text with some potential variations
    import random
    variations = [
        command_text,
        command_text.replace(" ", "  "),  # Extra spaces
        command_text + " ",  # Trailing space
        command_text.upper(),  # Uppercase
        command_text.lower(),  # Lowercase
    ]

    # Add some random noise simulation
    if random.random() < 0.1:  # 10% chance of "noise"
        return "noise detected"

    return random.choice(variations)

def run_recognition_tests():
    """Run comprehensive recognition tests"""
    print("Starting Voice Recognition Tests")
    print("=" * 50)

    # Import your command parser
    try:
        from command_parser import VoiceCommandParser
        parser = VoiceCommandParser()
    except ImportError:
        print("Command parser not found, using simulation mode")
        parser = None
        return

    # Initialize test runner
    runner = RecognitionTestRunner(parser)

    # Test different categories
    categories = [
        'basic_movement',
        'movement_with_distance',
        'navigation',
        'grasping',
        'inspection',
        'stop_commands',
        'complex_commands'
    ]

    all_results = []

    for category in categories:
        print(f"\nTesting category: {category}")
        print("-" * 30)

        category_results = runner.run_category_tests(category)
        all_results.extend(category_results)

        # Print category summary
        correct_count = sum(1 for r in category_results if r['overall_correct'])
        total_count = len(category_results)
        accuracy = correct_count / total_count if total_count > 0 else 0

        print(f"  Accuracy: {correct_count}/{total_count} ({accuracy:.2%})")

        # Print detailed results for incorrectly parsed commands
        for result in category_results:
            if not result['overall_correct']:
                print(f"    ❌ '{result['command_text']}' -> Expected: {result['expected_intent']}, Got: {result['parsed_intent']}")

    # Generate overall report
    overall_report = runner.generate_report(all_results)

    print("\n" + "=" * 50)
    print("OVERALL TEST RESULTS")
    print("=" * 50)
    print(f"Total tests: {overall_report['total_tests']}")
    print(f"Intent accuracy: {overall_report['intent_accuracy']:.2%}")
    print(f"Entity accuracy: {overall_report['entity_accuracy']:.2%}")
    print(f"Overall accuracy: {overall_report['overall_accuracy']:.2%}")
    print(f"Average confidence: {overall_report['average_confidence']:.3f}")

    return overall_report

def run_real_time_tests():
    """Run tests with real audio input"""
    print("Starting Real-time Voice Recognition Tests")
    print("=" * 50)

    # Import required modules
    try:
        import pyaudio
        import speech_recognition as sr
        from command_parser import VoiceCommandParser
    except ImportError as e:
        print(f"Required modules not installed: {e}")
        print("Install with: pip install speechrecognition pyaudio")
        return

    # Initialize components
    parser = VoiceCommandParser()
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # Adjust for ambient noise
    print("Adjusting for ambient noise...")
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
    print("Ready for real-time testing!")

    # Get test commands
    test_suite = VoiceCommandTestSuite()
    commands = test_suite.get_all_commands()

    results = []

    print(f"\nStarting real-time tests with {len(commands)} commands...")
    print("Speak each command when prompted.")
    print("Press Ctrl+C to stop testing early.\n")

    try:
        for i, (command_text, expected_intent, expected_entities) in enumerate(commands):
            print(f"\nTest {i+1}/{len(commands)}: '{command_text}'")
            print("Speak now...")

            # Listen for audio
            with microphone as source:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)

            # Recognize speech
            try:
                recognized_text = recognizer.recognize_google(audio)
                print(f"Recognized: '{recognized_text}'")

                # Parse the recognized command
                parsed_command = parser.parse_command(recognized_text)

                result = {
                    'command_text': command_text,
                    'expected_intent': expected_intent,
                    'expected_entities': expected_entities,
                    'recognized_text': recognized_text,
                    'parsed_intent': parsed_command.intent if parsed_command else None,
                    'parsed_entities': parsed_command.entities if parsed_command else {},
                    'confidence': parsed_command.confidence if parsed_command else 0.0,
                    'intent_correct': parsed_command.intent == expected_intent if parsed_command else False,
                    'overall_correct': False
                }

                # Check overall correctness
                if parsed_command:
                    result['overall_correct'] = (
                        result['intent_correct'] and
                        # Simplified entity comparison
                        all(k in parsed_command.entities and
                            str(parsed_command.entities[k]).lower() == str(v).lower()
                            for k, v in expected_entities.items() if v)
                    )

                results.append(result)

                status = "✅" if result['overall_correct'] else "❌"
                print(f"Result: {status} Intent: {result['parsed_intent']}, Confidence: {result['confidence']:.2f}")

            except sr.UnknownValueError:
                print("❌ Could not understand audio")
                results.append({
                    'command_text': command_text,
                    'expected_intent': expected_intent,
                    'expected_entities': expected_entities,
                    'recognized_text': '',
                    'parsed_intent': None,
                    'parsed_entities': {},
                    'confidence': 0.0,
                    'intent_correct': False,
                    'overall_correct': False
                })
            except sr.RequestError as e:
                print(f"❌ Recognition request failed: {e}")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

    except KeyboardInterrupt:
        print("\n\nTesting interrupted by user.")

    # Generate report
    correct_count = sum(1 for r in results if r['overall_correct'])
    total_count = len(results)

    print("\n" + "=" * 50)
    print("REAL-TIME TEST RESULTS")
    print("=" * 50)
    print(f"Successful recognitions: {correct_count}/{total_count}")
    print(f"Success rate: {correct_count/total_count:.2% if total_count > 0 else 0:.2%}")

    # Show failed cases
    failed_cases = [r for r in results if not r['overall_correct'] and r['recognized_text']]
    if failed_cases:
        print(f"\nFailed cases ({len(failed_cases)}):")
        for r in failed_cases[:10]:  # Show first 10
            print(f"  Expected: '{r['command_text']}' -> Got: '{r['recognized_text']}'")

def main():
    """Main function to run recognition tests"""
    print("Voice Recognition Testing Suite")
    print("Choose test type:")
    print("1. Simulated tests (using test commands)")
    print("2. Real-time tests (with microphone)")

    choice = input("Enter choice (1 or 2): ").strip()

    if choice == "1":
        run_recognition_tests()
    elif choice == "2":
        run_real_time_tests()
    else:
        print("Invalid choice. Running simulated tests...")
        run_recognition_tests()

if __name__ == "__main__":
    main()
```

### Step 3: Create Performance Evaluation Script

Create a file `evaluate_performance.py`:

```python
#!/usr/bin/env python3
"""
Performance evaluation for voice recognition system
Measures accuracy, latency, and resource usage
"""
import time
import psutil
import os
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Dict
from test_commands import RecognitionTestRunner

def measure_system_resources():
    """Measure current system resource usage"""
    return {
        'cpu_percent': psutil.cpu_percent(interval=1),
        'memory_percent': psutil.virtual_memory().percent,
        'memory_used_mb': psutil.virtual_memory().used / (1024 * 1024),
        'memory_total_mb': psutil.virtual_memory().total / (1024 * 1024)
    }

def run_performance_tests():
    """Run comprehensive performance tests"""
    print("Starting Performance Evaluation")
    print("=" * 50)

    # Import components
    try:
        from command_parser import VoiceCommandParser
        parser = VoiceCommandParser()
    except ImportError:
        print("Command parser not found")
        return

    runner = RecognitionTestRunner(parser)

    # Test data
    test_commands = [
        ("move forward", "move", {"direction": "forward"}),
        ("go to kitchen", "navigate", {"location": "kitchen"}),
        ("pick up the red ball", "grasp", {"object": "red ball"}),
        ("look at the book", "inspect", {"object": "book"}),
        ("stop immediately", "stop", {}),
    ] * 10  # Repeat to get better measurements

    # Performance metrics
    latencies = []
    resource_snapshots = []
    accuracy_results = []

    print(f"Running {len(test_commands)} performance tests...")

    for i, (command_text, expected_intent, expected_entities) in enumerate(test_commands):
        # Measure before
        start_time = time.time()
        resources_before = measure_system_resources()

        # Run the test
        result = runner.run_test(command_text, expected_intent, expected_entities)
        accuracy_results.append(result)

        # Measure after
        end_time = time.time()
        resources_after = measure_system_resources()

        # Calculate latency
        latency = end_time - start_time
        latencies.append(latency)

        # Store resource changes
        resource_change = {
            'latency': latency,
            'cpu_change': resources_after['cpu_percent'] - resources_before['cpu_percent'],
            'memory_change_mb': resources_after['memory_used_mb'] - resources_before['memory_used_mb']
        }
        resource_snapshots.append(resource_change)

        if (i + 1) % 10 == 0:
            print(f"Completed {i + 1}/{len(test_commands)} tests")

    # Calculate statistics
    avg_latency = np.mean(latencies)
    std_latency = np.std(latencies)
    min_latency = np.min(latencies)
    max_latency = np.max(latencies)

    avg_cpu_change = np.mean([r['cpu_change'] for r in resource_snapshots])
    avg_memory_change = np.mean([r['memory_change_mb'] for r in resource_snapshots])

    # Calculate accuracy
    correct_count = sum(1 for r in accuracy_results if r['overall_correct'])
    accuracy = correct_count / len(accuracy_results) if accuracy_results else 0

    print("\n" + "=" * 50)
    print("PERFORMANCE EVALUATION RESULTS")
    print("=" * 50)
    print(f"Total tests: {len(test_commands)}")
    print(f"Accuracy: {accuracy:.2%}")
    print(f"Average latency: {avg_latency:.4f}s")
    print(f"Latency std dev: {std_latency:.4f}s")
    print(f"Min latency: {min_latency:.4f}s")
    print(f"Max latency: {max_latency:.4f}s")
    print(f"Avg CPU change: {avg_cpu_change:.2f}%")
    print(f"Avg memory change: {avg_memory_change:.2f}MB")

    # Generate visualizations
    create_performance_charts(latencies, resource_snapshots, accuracy_results)

def create_performance_charts(latencies: List[float], resource_snapshots: List[Dict], accuracy_results: List[Dict]):
    """Create performance visualization charts"""
    try:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Latency distribution
        axes[0, 0].hist(latencies, bins=20, edgecolor='black')
        axes[0, 0].set_title('Latency Distribution')
        axes[0, 0].set_xlabel('Latency (seconds)')
        axes[0, 0].set_ylabel('Frequency')

        # CPU usage over time
        cpu_changes = [r['cpu_change'] for r in resource_snapshots]
        axes[0, 1].plot(cpu_changes)
        axes[0, 1].set_title('CPU Usage Changes')
        axes[0, 1].set_xlabel('Test Number')
        axes[0, 1].set_ylabel('CPU Change (%)')

        # Memory usage over time
        memory_changes = [r['memory_change_mb'] for r in resource_snapshots]
        axes[1, 0].plot(memory_changes)
        axes[1, 0].set_title('Memory Usage Changes')
        axes[1, 0].set_xlabel('Test Number')
        axes[1, 0].set_ylabel('Memory Change (MB)')

        # Accuracy by command type
        intents = [r['expected_intent'] for r in accuracy_results]
        correct = [r['overall_correct'] for r in accuracy_results]

        unique_intents = list(set(intents))
        intent_accuracy = {}
        for intent in unique_intents:
            intent_tests = [correct[i] for i, test_intent in enumerate(intents) if test_intent == intent]
            intent_accuracy[intent] = sum(intent_tests) / len(intent_tests) if intent_tests else 0

        axes[1, 1].bar(intent_accuracy.keys(), intent_accuracy.values())
        axes[1, 1].set_title('Accuracy by Command Type')
        axes[1, 1].set_xlabel('Command Type')
        axes[1, 1].set_ylabel('Accuracy')
        axes[1, 1].tick_params(axis='x', rotation=45)

        plt.tight_layout()
        plt.savefig('voice_recognition_performance.png', dpi=300, bbox_inches='tight')
        print("Performance chart saved as 'voice_recognition_performance.png'")

    except ImportError:
        print("Matplotlib not available, skipping chart generation")

def run_stress_test():
    """Run stress test with high volume of commands"""
    print("Starting Stress Test")
    print("=" * 30)

    try:
        from command_parser import VoiceCommandParser
        parser = VoiceCommandParser()
    except ImportError:
        print("Command parser not found")
        return

    runner = RecognitionTestRunner(parser)

    # Create a large number of test commands
    stress_commands = []
    base_commands = [
        ("move forward", "move", {"direction": "forward"}),
        ("go to kitchen", "navigate", {"location": "kitchen"}),
        ("pick up object", "grasp", {"object": "object"}),
    ]

    # Repeat commands many times
    for _ in range(100):  # 300 total commands
        stress_commands.extend(base_commands)

    print(f"Running stress test with {len(stress_commands)} commands...")

    start_time = time.time()
    results = []

    for i, (command_text, expected_intent, expected_entities) in enumerate(stress_commands):
        result = runner.run_test(command_text, expected_intent, expected_entities)
        results.append(result)

        if (i + 1) % 50 == 0:
            current_time = time.time()
            elapsed = current_time - start_time
            rate = (i + 1) / elapsed
            print(f"Processed {i + 1}/{len(stress_commands)} commands, "
                  f"rate: {rate:.2f} commands/sec")

    end_time = time.time()
    total_time = end_time - start_time
    avg_rate = len(stress_commands) / total_time if total_time > 0 else 0

    # Calculate accuracy
    correct_count = sum(1 for r in results if r['overall_correct'])
    accuracy = correct_count / len(results) if results else 0

    print("\n" + "=" * 30)
    print("STRESS TEST RESULTS")
    print("=" * 30)
    print(f"Total commands: {len(stress_commands)}")
    print(f"Total time: {total_time:.2f}s")
    print(f"Average rate: {avg_rate:.2f} commands/sec")
    print(f"Accuracy: {accuracy:.2%}")
    print(f"Commands per second: {avg_rate:.2f}")

def main():
    """Main function for performance evaluation"""
    print("Voice Recognition Performance Evaluation")
    print("Choose test type:")
    print("1. Performance tests (latency, resources, accuracy)")
    print("2. Stress test (high volume)")
    print("3. Both")

    choice = input("Enter choice (1, 2, or 3): ").strip()

    if choice == "1":
        run_performance_tests()
    elif choice == "2":
        run_stress_test()
    elif choice == "3":
        run_performance_tests()
        print("\n")
        run_stress_test()
    else:
        print("Invalid choice. Running performance tests...")
        run_performance_tests()

if __name__ == "__main__":
    main()
```

## Expected Outcomes

After completing this exercise, you should:

1. Test the voice recognition system with 50+ different command types
2. Evaluate recognition accuracy across different command categories
3. Measure system performance (latency, resource usage)
4. Identify areas for improvement in the recognition pipeline
5. Generate detailed reports on system performance

## Verification Steps

1. Run the test suite with various command types
2. Verify that the command parser correctly identifies intents and entities
3. Check that performance metrics are within acceptable ranges
4. Generate performance reports and charts
5. Identify and document any recognition failures

## Troubleshooting

- If recognition accuracy is low, check audio preprocessing parameters
- If latency is high, optimize the recognition pipeline
- If resource usage is excessive, consider algorithm optimization
- If certain command types consistently fail, update the parsing patterns