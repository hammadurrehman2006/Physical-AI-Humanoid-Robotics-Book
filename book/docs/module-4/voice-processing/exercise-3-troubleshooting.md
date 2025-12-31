# Exercise 3: Troubleshooting Common Voice Processing Issues

## Objective
Identify, diagnose, and resolve common issues in voice processing systems including audio input problems, recognition failures, and performance bottlenecks.

## Prerequisites
- Completed previous exercises
- Working voice processing system
- Understanding of system components

## Exercise Steps

### Step 1: Create Diagnostic Tools

Create a file `voice_diagnostics.py`:

```python
#!/usr/bin/env python3
"""
Voice processing diagnostic tools
Identifies and troubleshoots common issues
"""
import pyaudio
import numpy as np
import wave
import time
import subprocess
import sys
import os
from typing import Dict, List, Tuple, Optional
import json

class VoiceDiagnostics:
    """Diagnostic tools for voice processing system"""

    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.diagnostics = {}

    def check_audio_devices(self) -> Dict:
        """Check available audio devices"""
        print("Checking audio devices...")

        devices = {
            'input_devices': [],
            'output_devices': [],
            'default_input': None,
            'default_output': None
        }

        try:
            for i in range(self.audio.get_device_count()):
                device_info = self.audio.get_device_info_by_index(i)

                if device_info['maxInputChannels'] > 0:
                    devices['input_devices'].append({
                        'index': i,
                        'name': device_info['name'],
                        'max_channels': device_info['maxInputChannels'],
                        'default_sample_rate': device_info['defaultSampleRate']
                    })

                if device_info['maxOutputChannels'] > 0:
                    devices['output_devices'].append({
                        'index': i,
                        'name': device_info['name'],
                        'max_channels': device_info['maxOutputChannels'],
                        'default_sample_rate': device_info['defaultSampleRate']
                    })

            # Get default devices
            try:
                default_input = self.audio.get_default_input_device_info()
                devices['default_input'] = {
                    'index': default_input['index'],
                    'name': default_input['name']
                }
            except OSError:
                devices['default_input'] = None

            try:
                default_output = self.audio.get_default_output_device_info()
                devices['default_output'] = {
                    'index': default_output['index'],
                    'name': default_output['name']
                }
            except OSError:
                devices['default_output'] = None

        except Exception as e:
            print(f"Error checking audio devices: {e}")
            devices['error'] = str(e)

        return devices

    def test_audio_input(self, device_index: int = None, duration: float = 3.0) -> Dict:
        """Test audio input from specified device"""
        print(f"Testing audio input (device: {device_index or 'default'}, duration: {duration}s)...")

        result = {
            'success': False,
            'device_index': device_index,
            'duration': duration,
            'sample_rate': 44100,
            'channels': 1,
            'format': pyaudio.paInt16,
            'error': None,
            'audio_level': 0.0,
            'noise_level': 0.0,
            'clipping_detected': False
        }

        try:
            # Open stream
            stream = self.audio.open(
                format=result['format'],
                channels=result['channels'],
                rate=result['sample_rate'],
                input=True,
                input_device_index=device_index,
                frames_per_buffer=1024
            )

            print("Recording...")
            frames = []
            total_samples = 0
            max_amplitude = 0

            for _ in range(0, int(result['sample_rate'] / 1024 * duration)):
                data = stream.read(1024, exception_on_overflow=False)
                frames.append(data)

                # Convert to numpy array for analysis
                audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                # Calculate statistics
                total_samples += len(audio_data)
                current_max = np.max(np.abs(audio_data))
                if current_max > max_amplitude:
                    max_amplitude = current_max

                # Check for clipping
                if current_max >= 0.95:
                    result['clipping_detected'] = True

            # Calculate average audio level
            all_audio = np.concatenate([np.frombuffer(f, dtype=np.int16).astype(np.float32) / 32768.0 for f in frames])
            result['audio_level'] = float(np.mean(np.abs(all_audio)))
            result['noise_level'] = float(np.std(all_audio))

            # Clean up
            stream.stop_stream()
            stream.close()

            result['success'] = True
            print(f"Recording successful. Audio level: {result['audio_level']:.4f}, Max amplitude: {max_amplitude:.4f}")

        except Exception as e:
            result['error'] = str(e)
            print(f"Audio input test failed: {e}")

        return result

    def check_system_resources(self) -> Dict:
        """Check system resources"""
        print("Checking system resources...")

        try:
            import psutil

            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')

            # Check network connectivity
            network_connected = True
            try:
                import socket
                socket.create_connection(("8.8.8.8", 53), timeout=3)
            except OSError:
                network_connected = False

            resources = {
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'memory_available_gb': memory.available / (1024**3),
                'disk_percent': (disk.used / disk.total) * 100,
                'network_connected': network_connected
            }

            return resources

        except ImportError:
            print("psutil not available, skipping system resource check")
            return {'error': 'psutil not installed'}

    def check_dependencies(self) -> Dict:
        """Check required dependencies"""
        print("Checking dependencies...")

        dependencies = {
            'pyaudio': False,
            'numpy': False,
            'scipy': False,
            'librosa': False,
            'pydub': False,
            'vosk': False,
            'openai': False,
            'rclpy': False
        }

        # Check each dependency
        for dep in dependencies.keys():
            try:
                __import__(dep)
                dependencies[dep] = True
            except ImportError:
                dependencies[dep] = False

        return dependencies

    def check_environment_variables(self) -> Dict:
        """Check required environment variables"""
        print("Checking environment variables...")

        env_vars = {
            'OPENAI_API_KEY': os.getenv('OPENAI_API_KEY') is not None,
            'VOSK_MODEL_PATH': os.getenv('VOSK_MODEL_PATH') is not None,
            'PYTHONPATH': os.getenv('PYTHONPATH') is not None
        }

        return env_vars

    def run_complete_diagnostic(self) -> Dict:
        """Run complete diagnostic suite"""
        print("Running complete diagnostic...")

        diagnostic_results = {
            'timestamp': time.time(),
            'audio_devices': self.check_audio_devices(),
            'system_resources': self.check_system_resources(),
            'dependencies': self.check_dependencies(),
            'environment_variables': self.check_environment_variables(),
            'audio_test': None
        }

        # Run audio test on default input device if available
        if diagnostic_results['audio_devices']['default_input']:
            device_index = diagnostic_results['audio_devices']['default_input']['index']
            diagnostic_results['audio_test'] = self.test_audio_input(device_index=device_index)
        else:
            diagnostic_results['audio_test'] = {'error': 'No default input device found'}

        return diagnostic_results

    def generate_diagnostic_report(self, results: Dict) -> str:
        """Generate human-readable diagnostic report"""
        report = []
        report.append("=" * 60)
        report.append("VOICE PROCESSING DIAGNOSTIC REPORT")
        report.append("=" * 60)

        # Audio devices
        report.append("\nAUDIO DEVICES:")
        report.append("-" * 20)
        input_devices = results['audio_devices'].get('input_devices', [])
        output_devices = results['audio_devices'].get('output_devices', [])

        if input_devices:
            report.append("Input devices:")
            for device in input_devices:
                report.append(f"  [{device['index']}] {device['name']} (Channels: {device['max_channels']})")
        else:
            report.append("  No input devices found!")

        if output_devices:
            report.append("Output devices:")
            for device in output_devices:
                report.append(f"  [{device['index']}] {device['name']} (Channels: {device['max_channels']})")
        else:
            report.append("  No output devices found!")

        # System resources
        report.append("\nSYSTEM RESOURCES:")
        report.append("-" * 20)
        resources = results['system_resources']
        if 'error' not in resources:
            report.append(f"  CPU: {resources['cpu_percent']:.1f}%")
            report.append(f"  Memory: {resources['memory_percent']:.1f}%")
            report.append(f"  Available: {resources['memory_available_gb']:.1f}GB")
            report.append(f"  Network: {'Connected' if resources['network_connected'] else 'Disconnected'}")
        else:
            report.append(f"  Error: {resources['error']}")

        # Dependencies
        report.append("\nDEPENDENCIES:")
        report.append("-" * 20)
        deps = results['dependencies']
        for dep, installed in deps.items():
            status = "✓" if installed else "✗"
            report.append(f"  {status} {dep}")

        # Environment variables
        report.append("\nENVIRONMENT VARIABLES:")
        report.append("-" * 20)
        env_vars = results['environment_variables']
        for var, present in env_vars.items():
            status = "✓" if present else "✗"
            report.append(f"  {status} {var}")

        # Audio test
        report.append("\nAUDIO TEST:")
        report.append("-" * 20)
        audio_test = results['audio_test']
        if audio_test and 'error' not in audio_test:
            report.append(f"  Success: {audio_test['success']}")
            report.append(f"  Audio level: {audio_test['audio_level']:.4f}")
            report.append(f"  Noise level: {audio_test['noise_level']:.4f}")
            report.append(f"  Clipping detected: {audio_test['clipping_detected']}")
        else:
            report.append(f"  Error: {audio_test.get('error', 'Unknown error')}")

        report.append("=" * 60)

        return "\n".join(report)

    def cleanup(self):
        """Clean up resources"""
        self.audio.terminate()

class IssueDetector:
    """Detects common issues in voice processing systems"""

    def __init__(self):
        self.issues = []

    def analyze_diagnostics(self, diagnostic_results: Dict) -> List[Dict]:
        """Analyze diagnostic results to detect issues"""
        self.issues = []

        # Check audio devices
        audio_devices = diagnostic_results.get('audio_devices', {})
        input_devices = audio_devices.get('input_devices', [])

        if not input_devices:
            self.issues.append({
                'severity': 'critical',
                'category': 'audio',
                'issue': 'No audio input devices found',
                'description': 'The system cannot find any audio input devices. Check microphone connections and drivers.',
                'solution': '1. Check if microphone is physically connected\n2. Verify microphone permissions\n3. Install/reinstall audio drivers\n4. Check system audio settings'
            })

        # Check system resources
        resources = diagnostic_results.get('system_resources', {})
        if 'error' not in resources:
            if resources.get('cpu_percent', 0) > 90:
                self.issues.append({
                    'severity': 'warning',
                    'category': 'performance',
                    'issue': 'High CPU usage',
                    'description': 'CPU usage is very high, which may affect real-time audio processing.',
                    'solution': 'Close unnecessary applications, consider upgrading hardware, or optimize audio processing pipeline.'
                })

            if resources.get('memory_percent', 0) > 90:
                self.issues.append({
                    'severity': 'warning',
                    'category': 'performance',
                    'issue': 'High memory usage',
                    'description': 'Memory usage is very high, which may cause audio processing issues.',
                    'solution': 'Close unnecessary applications or increase system memory.'
                })

        # Check dependencies
        deps = diagnostic_results.get('dependencies', {})
        critical_deps = ['pyaudio', 'numpy', 'openai']

        for dep in critical_deps:
            if not deps.get(dep, False):
                self.issues.append({
                    'severity': 'critical',
                    'category': 'dependencies',
                    'issue': f'Missing critical dependency: {dep}',
                    'description': f'The required package {dep} is not installed.',
                    'solution': f'Install the package using: pip install {dep}'
                })

        # Check environment variables
        env_vars = diagnostic_results.get('environment_variables', {})
        if not env_vars.get('OPENAI_API_KEY', False):
            self.issues.append({
                'severity': 'warning',
                'category': 'configuration',
                'issue': 'OpenAI API key not set',
                'description': 'OpenAI API key is not configured, cloud recognition will not work.',
                'solution': 'Set the OPENAI_API_KEY environment variable with a valid API key.'
            })

        # Check audio test results
        audio_test = diagnostic_results.get('audio_test', {})
        if audio_test.get('error'):
            self.issues.append({
                'severity': 'critical',
                'category': 'audio',
                'issue': 'Audio input test failed',
                'description': f'Audio input test failed: {audio_test["error"]}',
                'solution': 'Check microphone connections, permissions, and drivers.'
            })
        elif not audio_test.get('success', False):
            self.issues.append({
                'severity': 'critical',
                'category': 'audio',
                'issue': 'Audio input not working',
                'description': 'Audio input is not functioning properly.',
                'solution': 'Check microphone connections and system audio settings.'
            })
        elif audio_test.get('audio_level', 0) < 0.001:
            self.issues.append({
                'severity': 'warning',
                'category': 'audio',
                'issue': 'Very low audio level detected',
                'description': f'Audio level is very low ({audio_test["audio_level"]:.4f}), microphone may be too quiet.',
                'solution': 'Increase microphone gain in system settings or move closer to microphone.'
            })
        elif audio_test.get('clipping_detected', False):
            self.issues.append({
                'severity': 'warning',
                'category': 'audio',
                'issue': 'Audio clipping detected',
                'description': 'Audio is clipping, indicating possible distortion.',
                'solution': 'Reduce microphone gain in system settings to prevent clipping.'
            })

        return self.issues

    def categorize_issues(self) -> Dict[str, List[Dict]]:
        """Categorize issues by severity"""
        categorized = {
            'critical': [],
            'warning': [],
            'info': []
        }

        for issue in self.issues:
            severity = issue['severity']
            if severity in categorized:
                categorized[severity].append(issue)

        return categorized

    def generate_issue_report(self) -> str:
        """Generate issue report"""
        if not self.issues:
            return "No issues detected!"

        report = []
        report.append("ISSUE DETECTION REPORT")
        report.append("=" * 40)

        categorized = self.categorize_issues()

        for severity, issues in categorized.items():
            if issues:
                report.append(f"\n{severity.upper()} ISSUES ({len(issues)}):")
                report.append("-" * 20)
                for i, issue in enumerate(issues, 1):
                    report.append(f"{i}. {issue['issue']}")
                    report.append(f"   Description: {issue['description']}")
                    report.append(f"   Solution: {issue['solution']}")
                    report.append("")

        return "\n".join(report)

def main():
    """Run diagnostic tools"""
    print("Voice Processing Diagnostic Tool")
    print("=" * 40)

    # Create diagnostic instance
    diagnostics = VoiceDiagnostics()

    try:
        # Run complete diagnostic
        results = diagnostics.run_complete_diagnostic()

        # Print diagnostic report
        report = diagnostics.generate_diagnostic_report(results)
        print(report)

        # Detect issues
        detector = IssueDetector()
        issues = detector.analyze_diagnostics(results)

        # Print issue report
        issue_report = detector.generate_issue_report()
        print(issue_report)

        # Save results to file
        with open('diagnostic_results.json', 'w') as f:
            json.dump(results, f, indent=2, default=str)

        print("Diagnostic results saved to 'diagnostic_results.json'")

    finally:
        # Clean up
        diagnostics.cleanup()

if __name__ == "__main__":
    main()
```

### Step 2: Create Troubleshooting Scenarios

Create a file `troubleshooting_scenarios.py`:

```python
#!/usr/bin/env python3
"""
Troubleshooting scenarios for voice processing issues
"""
import time
import random
from typing import Dict, List, Callable
import numpy as np

class TroubleshootingScenario:
    """A troubleshooting scenario with symptoms, causes, and solutions"""

    def __init__(self, name: str, description: str, symptoms: List[str],
                 causes: List[str], solutions: List[str], difficulty: str = "medium"):
        self.name = name
        self.description = description
        self.symptoms = symptoms
        self.causes = causes
        self.solutions = solutions
        self.difficulty = difficulty

    def simulate_issue(self) -> Dict:
        """Simulate the issue for troubleshooting practice"""
        return {
            'name': self.name,
            'description': self.description,
            'symptoms': self.symptoms,
            'difficulty': self.difficulty
        }

class VoiceTroubleshootingSimulator:
    """Simulates voice processing issues for troubleshooting practice"""

    def __init__(self):
        self.scenarios = self._create_scenarios()

    def _create_scenarios(self) -> List[TroubleshootingScenario]:
        """Create troubleshooting scenarios"""
        return [
            TroubleshootingScenario(
                name="No Audio Input",
                description="System cannot capture audio from microphone",
                symptoms=[
                    "No audio data being received",
                    "Audio input node not publishing messages",
                    "Microphone LED not lighting up",
                    "Audio test fails"
                ],
                causes=[
                    "Microphone not connected properly",
                    "Audio driver issues",
                    "Microphone permissions not granted",
                    "Audio device not selected as default",
                    "Hardware failure"
                ],
                solutions=[
                    "Check physical connections",
                    "Verify microphone permissions in system settings",
                    "Update/reinstall audio drivers",
                    "Select correct audio input device",
                    "Test with different microphone"
                ],
                difficulty="easy"
            ),
            TroubleshootingScenario(
                name="Poor Recognition Quality",
                description="Voice recognition accuracy is very low",
                symptoms=[
                    "Commands not recognized correctly",
                    "High error rate in transcription",
                    "Similar commands interpreted differently",
                    "Background noise not filtered properly"
                ],
                causes=[
                    "High background noise",
                    "Microphone too far from speaker",
                    "Audio preprocessing not configured properly",
                    "Whisper API key issues",
                    "Incorrect language settings"
                ],
                solutions=[
                    "Improve acoustic environment",
                    "Move microphone closer to speaker",
                    "Adjust noise reduction parameters",
                    "Verify OpenAI API key and quota",
                    "Set correct language in configuration"
                ],
                difficulty="medium"
            ),
            TroubleshootingScenario(
                name="High Latency",
                description="Long delay between speaking and command execution",
                symptoms=[
                    "Noticeable delay in command processing",
                    "Real-time response requirements not met",
                    "Buffer overflow warnings",
                    "Commands arriving after timeout"
                ],
                causes=[
                    "Network latency to cloud services",
                    "CPU overload from audio processing",
                    "Large audio buffer sizes",
                    "Inefficient preprocessing algorithms",
                    "Concurrent system processes"
                ],
                solutions=[
                    "Optimize audio buffer sizes",
                    "Use local recognition when possible",
                    "Implement asynchronous processing",
                    "Reduce preprocessing complexity",
                    "Close unnecessary applications"
                ],
                difficulty="hard"
            ),
            TroubleshootingScenario(
                name="Intermittent Failures",
                description="Voice processing works sometimes but fails randomly",
                symptoms=[
                    "Intermittent recognition failures",
                    "Audio stream occasionally stops",
                    "Commands work at times, fail at others",
                    "Variable performance throughout the day"
                ],
                causes=[
                    "Unstable network connection",
                    "Resource contention with other processes",
                    "Temperature-related hardware issues",
                    "Memory leaks in audio processing",
                    "Driver instability"
                ],
                solutions=[
                    "Monitor network stability",
                    "Check system resource usage over time",
                    "Implement proper error handling and recovery",
                    "Add memory monitoring and cleanup",
                    "Update drivers to latest stable version"
                ],
                difficulty="hard"
            ),
            TroubleshootingScenario(
                name="Resource Exhaustion",
                description="System runs out of CPU, memory, or other resources",
                symptoms=[
                    "High CPU usage",
                    "Memory usage constantly increasing",
                    "Audio processing thread stops",
                    "System becomes unresponsive"
                ],
                causes=[
                    "Inefficient audio processing algorithms",
                    "Memory leaks in audio buffers",
                    "Too many concurrent processing tasks",
                    "Unoptimized preprocessing pipeline",
                    "Insufficient system resources"
                ],
                solutions=[
                    "Optimize audio processing algorithms",
                    "Implement proper memory management",
                    "Limit concurrent processing tasks",
                    "Use more efficient data structures",
                    "Upgrade system hardware if necessary"
                ],
                difficulty="medium"
            ),
            TroubleshootingScenario(
                name="Configuration Issues",
                description="System not responding to configuration changes",
                symptoms=[
                    "Configuration parameters not taking effect",
                    "Default settings used despite changes",
                    "Parameter validation errors",
                    "Configuration file not loading"
                ],
                causes=[
                    "Configuration file syntax errors",
                    "Parameters overridden by code defaults",
                    "Environment variables not set correctly",
                    "Configuration loading order issues",
                    "Parameter validation failures"
                ],
                solutions=[
                    "Validate configuration file syntax",
                    "Check parameter precedence order",
                    "Verify environment variable names",
                    "Review configuration loading code",
                    "Add configuration validation logging"
                ],
                difficulty="easy"
            )
        ]

    def get_scenario_by_difficulty(self, difficulty: str) -> List[TroubleshootingScenario]:
        """Get scenarios of specific difficulty"""
        return [s for s in self.scenarios if s.difficulty == difficulty]

    def run_scenario(self, scenario: TroubleshootingScenario) -> Dict:
        """Run a troubleshooting scenario"""
        print(f"\nTROUBLESHOOTING SCENARIO: {scenario.name}")
        print("=" * 50)
        print(f"Description: {scenario.description}")
        print(f"Difficulty: {scenario.difficulty}")
        print("\nSymptoms:")
        for i, symptom in enumerate(scenario.symptoms, 1):
            print(f"  {i}. {symptom}")

        print("\nWhat would you check first?")
        print("Type 'causes' to see possible causes")
        print("Type 'solutions' to see solutions")
        print("Type 'next' for next scenario")

        return scenario.simulate_issue()

    def practice_troubleshooting(self):
        """Run troubleshooting practice session"""
        print("Voice Processing Troubleshooting Practice")
        print("=" * 50)

        # Shuffle scenarios for practice
        scenarios = self.scenarios.copy()
        random.shuffle(scenarios)

        for scenario in scenarios:
            self.run_scenario(scenario)

            while True:
                user_input = input("\nEnter command: ").strip().lower()

                if user_input == 'causes':
                    print(f"\nPossible Causes for {scenario.name}:")
                    for i, cause in enumerate(scenario.causes, 1):
                        print(f"  {i}. {cause}")

                elif user_input == 'solutions':
                    print(f"\nSolutions for {scenario.name}:")
                    for i, solution in enumerate(scenario.solutions, 1):
                        print(f"  {i}. {solution}")
                    break  # Move to next scenario

                elif user_input == 'next':
                    break

                else:
                    print("Commands: 'causes', 'solutions', 'next'")

class RealWorldTroubleshootingGuide:
    """Provides systematic approach to real-world troubleshooting"""

    def __init__(self):
        self.troubleshooting_steps = [
            {
                'step': 1,
                'name': 'Identify the Problem',
                'description': 'Clearly define what is not working',
                'actions': [
                    'Observe symptoms',
                    'Check error messages',
                    'Determine when problem started',
                    'Identify affected components'
                ]
            },
            {
                'step': 2,
                'name': 'Gather Information',
                'description': 'Collect relevant data about the issue',
                'actions': [
                    'Run diagnostic tools',
                    'Check system logs',
                    'Monitor resource usage',
                    'Test with minimal configuration'
                ]
            },
            {
                'step': 3,
                'name': 'Formulate Hypothesis',
                'description': 'Develop possible explanations for the issue',
                'actions': [
                    'List possible causes',
                    'Prioritize by likelihood',
                    'Consider recent changes',
                    'Check for patterns'
                ]
            },
            {
                'step': 4,
                'name': 'Test Hypothesis',
                'description': 'Systematically test each possible cause',
                'actions': [
                    'Make one change at a time',
                    'Test after each change',
                    'Document results',
                    'Revert changes that don\'t help'
                ]
            },
            {
                'step': 5,
                'name': 'Implement Solution',
                'description': 'Apply the fix that resolves the issue',
                'actions': [
                    'Apply the working solution',
                    'Verify the fix works',
                    'Test related functionality',
                    'Monitor for side effects'
                ]
            },
            {
                'step': 6,
                'name': 'Document and Prevent',
                'description': 'Record the solution and prevent recurrence',
                'actions': [
                    'Document the issue and solution',
                    'Update troubleshooting guides',
                    'Implement preventive measures',
                    'Share knowledge with team'
                ]
            }
        ]

    def get_systematic_approach(self) -> List[Dict]:
        """Get the systematic troubleshooting approach"""
        return self.troubleshooting_steps

    def apply_to_scenario(self, scenario: TroubleshootingScenario) -> List[Dict]:
        """Apply systematic approach to a specific scenario"""
        approach = []

        for step in self.troubleshooting_steps:
            step_info = step.copy()
            step_info['scenario_application'] = self._apply_step_to_scenario(step['name'], scenario)
            approach.append(step_info)

        return approach

    def _apply_step_to_scenario(self, step_name: str, scenario: TroubleshootingScenario) -> str:
        """Apply a troubleshooting step to a specific scenario"""
        if step_name == "Identify the Problem":
            return f"Problem: {scenario.description}. Symptoms: {', '.join(scenario.symptoms[:2])}..."
        elif step_name == "Gather Information":
            return f"Run diagnostics, check logs for {scenario.name} related errors, monitor audio input levels"
        elif step_name == "Formulate Hypothesis":
            return f"Likely causes: {', '.join(scenario.causes[:2])}"
        elif step_name == "Test Hypothesis":
            return f"Test solutions: {', '.join(scenario.solutions[:2])}"
        elif step_name == "Implement Solution":
            return f"Apply: {scenario.solutions[0]}"
        elif step_name == "Document and Prevent":
            return f"Document fix for {scenario.name}, add to troubleshooting guide"

        return "Apply systematic approach to this scenario"

def main():
    """Run troubleshooting practice"""
    print("Voice Processing Troubleshooting Simulator")
    print("=" * 50)

    simulator = VoiceTroubleshootingSimulator()
    guide = RealWorldTroubleshootingGuide()

    print("Available scenarios:")
    for i, scenario in enumerate(simulator.scenarios, 1):
        print(f"{i}. {scenario.name} ({scenario.difficulty})")

    print("\nChoose troubleshooting mode:")
    print("1. Practice scenarios")
    print("2. Systematic approach guide")
    print("3. Run all scenarios")

    choice = input("Enter choice (1-3): ").strip()

    if choice == "1":
        simulator.practice_troubleshooting()
    elif choice == "2":
        print("\nSYSTEMATIC TROUBLESHOOTING APPROACH:")
        print("=" * 50)
        for step in guide.get_systematic_approach():
            print(f"\nStep {step['step']}: {step['name']}")
            print(f"Description: {step['description']}")
            print("Actions:")
            for action in step['actions']:
                print(f"  - {action}")
    elif choice == "3":
        for scenario in simulator.scenarios:
            print(f"\nApplying systematic approach to: {scenario.name}")
            approach = guide.apply_to_scenario(scenario)
            for step in approach:
                print(f"  {step['name']}: {step['scenario_application']}")
    else:
        print("Invalid choice. Running practice scenarios...")
        simulator.practice_troubleshooting()

if __name__ == "__main__":
    main()
```

### Step 3: Create Debugging Tools

Create a file `debugging_tools.py`:

```python
#!/usr/bin/env python3
"""
Debugging tools for voice processing systems
"""
import logging
import threading
import time
import queue
import numpy as np
from typing import Dict, List, Callable, Any
import json
import os
from datetime import datetime

class VoiceDebugger:
    """Advanced debugging tools for voice processing"""

    def __init__(self, log_file: str = "voice_processing_debug.log"):
        self.log_file = log_file
        self.setup_logging()
        self.debug_queues = {}
        self.metrics = {}
        self.callbacks = {}

    def setup_logging(self):
        """Setup logging configuration"""
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(self.log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)

    def log_audio_data(self, audio_chunk: np.ndarray, source: str = "unknown"):
        """Log audio data for debugging"""
        if audio_chunk is not None:
            metrics = {
                'timestamp': time.time(),
                'source': source,
                'shape': audio_chunk.shape,
                'dtype': str(audio_chunk.dtype),
                'mean': float(np.mean(audio_chunk)),
                'std': float(np.std(audio_chunk)),
                'max': float(np.max(audio_chunk)),
                'min': float(np.min(audio_chunk)),
                'rms': float(np.sqrt(np.mean(audio_chunk ** 2))),
                'zero_crossings': int(np.sum(audio_chunk[:-1] * audio_chunk[1:] < 0))
            }

            self.logger.debug(f"Audio data from {source}: {metrics}")
            return metrics
        return None

    def log_processing_step(self, step_name: str, data: Any, metadata: Dict = None):
        """Log processing step with data"""
        log_entry = {
            'timestamp': time.time(),
            'step': step_name,
            'data_type': type(data).__name__,
            'metadata': metadata or {}
        }

        if isinstance(data, np.ndarray):
            log_entry['data_shape'] = data.shape
            log_entry['data_dtype'] = str(data.dtype)
        else:
            log_entry['data_repr'] = str(data)[:200]  # Limit length

        self.logger.debug(f"Processing step: {log_entry}")
        return log_entry

    def create_debug_queue(self, name: str):
        """Create a debug queue to track data flow"""
        self.debug_queues[name] = {
            'queue': queue.Queue(),
            'items': [],
            'stats': {
                'count': 0,
                'last_access': time.time(),
                'avg_size': 0
            }
        }

    def add_to_debug_queue(self, queue_name: str, item: Any):
        """Add item to debug queue"""
        if queue_name in self.debug_queues:
            self.debug_queues[queue_name]['queue'].put(item)
            self.debug_queues[queue_name]['items'].append(item)
            self.debug_queues[queue_name]['stats']['count'] += 1
            self.debug_queues[queue_name]['stats']['last_access'] = time.time()

            # Log queue metrics periodically
            if self.debug_queues[queue_name]['stats']['count'] % 10 == 0:
                self.log_queue_metrics(queue_name)

    def log_queue_metrics(self, queue_name: str):
        """Log metrics for a debug queue"""
        if queue_name in self.debug_queues:
            stats = self.debug_queues[queue_name]['stats']
            queue_size = self.debug_queues[queue_name]['queue'].qsize()
            stats['avg_size'] = (stats['avg_size'] * (stats['count'] // 10 - 1) + queue_size) / (stats['count'] // 10)

            self.logger.info(f"Queue {queue_name}: count={stats['count']}, current_size={queue_size}, avg_size={stats['avg_size']:.2f}")

    def register_callback(self, name: str, callback: Callable):
        """Register a callback for debugging"""
        self.callbacks[name] = callback

    def trigger_callback(self, name: str, *args, **kwargs):
        """Trigger a registered callback"""
        if name in self.callbacks:
            try:
                return self.callbacks[name](*args, **kwargs)
            except Exception as e:
                self.logger.error(f"Callback {name} failed: {e}")
                return None
        return None

    def collect_system_metrics(self) -> Dict:
        """Collect system metrics for debugging"""
        try:
            import psutil
            import os

            process = psutil.Process(os.getpid())

            metrics = {
                'timestamp': time.time(),
                'cpu_percent': psutil.cpu_percent(),
                'memory_percent': psutil.virtual_memory().percent,
                'process_memory_mb': process.memory_info().rss / 1024 / 1024,
                'disk_percent': psutil.disk_usage('/').percent,
                'threads': process.num_threads(),
                'connections': len(process.connections()),
                'open_files': len(process.open_files()) if process.open_files() else 'N/A'
            }

            self.metrics.update(metrics)
            return metrics

        except ImportError:
            return {'error': 'psutil not available'}

    def start_performance_monitoring(self, interval: float = 1.0):
        """Start background performance monitoring"""
        def monitor():
            while getattr(self, 'monitoring_active', True):
                metrics = self.collect_system_metrics()
                if 'error' not in metrics:
                    self.logger.debug(f"System metrics: {metrics}")
                time.sleep(interval)

        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=monitor, daemon=True)
        self.monitoring_thread.start()

    def stop_performance_monitoring(self):
        """Stop performance monitoring"""
        self.monitoring_active = False
        if hasattr(self, 'monitoring_thread'):
            self.monitoring_thread.join(timeout=2.0)

    def generate_debug_report(self) -> str:
        """Generate a comprehensive debug report"""
        report = []
        report.append("VOICE PROCESSING DEBUG REPORT")
        report.append("=" * 50)
        report.append(f"Generated: {datetime.now().isoformat()}")

        # System metrics
        if self.metrics:
            report.append("\nSYSTEM METRICS:")
            report.append("-" * 20)
            for key, value in self.metrics.items():
                report.append(f"  {key}: {value}")

        # Debug queues
        if self.debug_queues:
            report.append("\nDEBUG QUEUES:")
            report.append("-" * 20)
            for name, queue_info in self.debug_queues.items():
                stats = queue_info['stats']
                report.append(f"  {name}: {stats['count']} items, last_access {time.time() - stats['last_access']:.1f}s ago")

        # Log file info
        if os.path.exists(self.log_file):
            file_size = os.path.getsize(self.log_file)
            report.append(f"\nLOG FILE: {self.log_file} ({file_size} bytes)")

        report.append("=" * 50)

        return "\n".join(report)

    def save_debug_data(self, filename: str):
        """Save debug data to file"""
        debug_data = {
            'timestamp': time.time(),
            'system_metrics': self.metrics,
            'debug_queues': {
                name: {
                    'stats': info['stats'],
                    'sample_items': info['items'][:10]  # Sample first 10 items
                } for name, info in self.debug_queues.items()
            },
            'log_file': self.log_file
        }

        with open(filename, 'w') as f:
            json.dump(debug_data, f, indent=2, default=str)

class AudioPipelineDebugger:
    """Specialized debugger for audio processing pipeline"""

    def __init__(self):
        self.debugger = VoiceDebugger()
        self.pipeline_metrics = {
            'audio_input_rate': 0,
            'processing_rate': 0,
            'latency': 0,
            'buffer_size': 0,
            'drop_rate': 0
        }
        self.start_time = time.time()
        self.audio_count = 0
        self.processed_count = 0
        self.last_audio_time = time.time()

    def log_audio_input(self, audio_data: np.ndarray):
        """Log audio input with timing"""
        current_time = time.time()
        self.audio_count += 1

        # Calculate input rate
        elapsed = current_time - self.start_time
        self.pipeline_metrics['audio_input_rate'] = self.audio_count / elapsed if elapsed > 0 else 0

        # Calculate inter-arrival time
        if current_time - self.last_audio_time > 0.1:  # 100ms threshold for drops
            self.pipeline_metrics['drop_rate'] += 1

        self.last_audio_time = current_time

        return self.debugger.log_audio_data(audio_data, "audio_input")

    def log_processing_result(self, input_metrics: Dict, result: Any):
        """Log processing result"""
        current_time = time.time()
        self.processed_count += 1

        # Calculate processing rate
        elapsed = current_time - self.start_time
        self.pipeline_metrics['processing_rate'] = self.processed_count / elapsed if elapsed > 0 else 0

        # Calculate latency if input metrics available
        if input_metrics:
            latency = current_time - input_metrics['timestamp']
            self.pipeline_metrics['latency'] = latency

        processing_metrics = {
            'input_timestamp': input_metrics.get('timestamp') if input_metrics else None,
            'result_type': type(result).__name__,
            'processing_timestamp': current_time,
            'latency': self.pipeline_metrics['latency']
        }

        self.debugger.logger.debug(f"Processing result: {processing_metrics}")
        return processing_metrics

    def get_pipeline_status(self) -> Dict:
        """Get current pipeline status"""
        current_time = time.time()
        elapsed = current_time - self.start_time

        status = self.pipeline_metrics.copy()
        status['uptime'] = elapsed
        status['total_audio'] = self.audio_count
        status['total_processed'] = self.processed_count
        status['success_rate'] = self.processed_count / self.audio_count if self.audio_count > 0 else 0

        return status

    def generate_pipeline_report(self) -> str:
        """Generate pipeline-specific debug report"""
        status = self.get_pipeline_status()

        report = []
        report.append("AUDIO PIPELINE DEBUG REPORT")
        report.append("=" * 50)
        report.append(f"Uptime: {status['uptime']:.2f}s")
        report.append(f"Total audio chunks: {status['total_audio']}")
        report.append(f"Total processed: {status['total_processed']}")
        report.append(f"Success rate: {status['success_rate']:.2%}")

        report.append(f"\nRATE METRICS:")
        report.append(f"  Audio input rate: {status['audio_input_rate']:.2f} Hz")
        report.append(f"  Processing rate: {status['processing_rate']:.2f} Hz")
        report.append(f"  Drop rate: {status['drop_rate']:.2f} per second")

        report.append(f"\nPERFORMANCE METRICS:")
        report.append(f"  Average latency: {status['latency']:.4f}s")

        return "\n".join(report)

def main():
    """Run debugging tools demonstration"""
    print("Voice Processing Debugging Tools")
    print("=" * 40)

    # Create debugger instance
    debugger = VoiceDebugger()
    pipeline_debugger = AudioPipelineDebugger()

    # Start performance monitoring
    debugger.start_performance_monitoring(interval=2.0)

    # Create debug queues
    debugger.create_debug_queue("audio_input")
    debugger.create_debug_queue("preprocessing")
    debugger.create_debug_queue("recognition")

    print("Simulating audio processing for debugging...")

    # Simulate some audio processing
    for i in range(20):
        # Simulate audio chunk
        sample_rate = 44100
        chunk_size = 1024
        t = np.linspace(0, chunk_size/sample_rate, chunk_size)
        audio_chunk = 0.1 * np.sin(2 * np.pi * 440 * t) + 0.01 * np.random.randn(len(t))

        # Log audio input
        input_metrics = pipeline_debugger.log_audio_input(audio_chunk)
        debugger.log_audio_data(audio_chunk, f"chunk_{i}")
        debugger.add_to_debug_queue("audio_input", audio_chunk)

        # Simulate preprocessing
        preprocessed = audio_chunk * 0.9  # Simple gain adjustment
        debugger.log_processing_step("preprocessing", preprocessed)
        debugger.add_to_debug_queue("preprocessing", preprocessed)

        # Simulate recognition result
        result = f"Command {i} recognized" if i % 3 != 2 else None  # Simulate some failures
        pipeline_debugger.log_processing_result(input_metrics, result)
        debugger.add_to_debug_queue("recognition", result)

        # Add some callbacks for demonstration
        if 'recognition_callback' not in debugger.callbacks:
            debugger.register_callback('recognition_callback', lambda r: f"Processed: {r}")

        if result:
            debugger.trigger_callback('recognition_callback', result)

        time.sleep(0.1)  # Simulate real-time processing

    # Generate reports
    print("\n" + debugger.generate_debug_report())
    print("\n" + pipeline_debugger.generate_pipeline_report())

    # Save debug data
    debugger.save_debug_data("debug_session.json")
    print("\nDebug data saved to 'debug_session.json'")

    # Stop monitoring
    debugger.stop_performance_monitoring()

    print(f"\nDebug log available at: {debugger.log_file}")

if __name__ == "__main__":
    main()
```

## Expected Outcomes

After completing this exercise, you should:

1. Identify common voice processing issues using diagnostic tools
2. Apply systematic troubleshooting approaches to resolve problems
3. Use debugging tools to analyze system behavior
4. Practice troubleshooting with simulated scenarios
5. Generate comprehensive debug reports

## Verification Steps

1. Run diagnostic tools to identify system issues
2. Practice troubleshooting with different scenarios
3. Use debugging tools during voice processing
4. Generate and analyze debug reports
5. Apply systematic approaches to real problems

## Troubleshooting

- If diagnostic tools don't work, check system permissions and dependencies
- If audio devices aren't detected, verify drivers and connections
- If debugging tools show high resource usage, optimize processing pipeline
- If issues persist, use the systematic troubleshooting approach to identify root causes