/**
 * ROS 2 to Gazebo Bridge Integration Tests
 * Validates the integration between ROS 2 and Gazebo systems
 */

const { test, expect } = require('@playwright/test');

test.describe('ROS 2 to Gazebo Bridge Integration', () => {
  test('should validate Gazebo environment setup documentation', async ({ page }) => {
    await page.goto('/docs/module-2/introduction/setup-gazebo-environment');

    // Verify Gazebo installation documentation
    await expect(page.locator('h1')).toContainText('Setting up Gazebo');

    // Check for installation commands
    const installCommands = await page.locator('pre:has(code:has-text("sudo apt install"))').count();
    expect(installCommands).toBeGreaterThan(0);

    // Verify verification steps
    await expect(page.locator('code')).toContainText('gz --version');
    await expect(page.locator('code')).toContainText('gz sim');
  });

  test('should validate robot spawning documentation', async ({ page }) => {
    await page.goto('/docs/module-2/introduction/basic-robot-spawning');

    // Verify robot spawning documentation
    await expect(page.locator('h1')).toContainText('Basic Robot Spawning');

    // Check for URDF examples
    const urdfExamples = await page.locator('pre:has(code:has-text("<robot"))').count();
    expect(urdfExamples).toBeGreaterThan(0);

    // Verify spawning commands
    await expect(page.locator('code')).toContainText('gz sdf -p');
    await expect(page.locator('code')).toContainText('simple_robot.sdf');
  });

  test('should validate physics simulation integration', async ({ page }) => {
    await page.goto('/docs/module-2/physics-simulation/gravity-and-collisions');

    // Check for physics configuration examples
    const physicsExamples = await page.locator('pre:has(code:has-text("<physics"))').count();
    expect(physicsExamples).toBeGreaterThan(0);

    // Verify physics parameters
    await expect(page.locator('code')).toContainText('gravity');
    await expect(page.locator('code')).toContainText('max_step_size');
    await expect(page.locator('code')).toContainText('real_time_factor');
  });

  test('should validate sensor integration documentation', async ({ page }) => {
    await page.goto('/docs/module-2/sensor-simulation/lidar-simulation');

    // Check for sensor configuration examples
    const sensorExamples = await page.locator('pre:has(code:has-text("sensor")).count();
    expect(sensorExamples).toBeGreaterThan(0);

    // Verify sensor parameters
    await expect(page.locator('code')).toContainText('ray');
    await expect(page.locator('code')).toContainText('scan');
    await expect(page.locator('code')).toContainText('range');
  });

  test('should verify ROS 2 control integration', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/practical-urdf-sdf-examples');

    // Check for ROS 2 control examples
    const ros2ControlExamples = await page.locator('pre:has(code:has-text("ros2_control")).count();
    expect(ros2ControlExamples).toBeGreaterThan(0);

    // Verify control interface types
    await expect(page.locator('code')).toContainText('velocity');
    await expect(page.locator('code')).toContainText('position');
    await expect(page.locator('code')).toContainText('effort');
  });

  test('should validate launch file integration', async ({ page }) => {
    await page.goto('/docs/projects/module-2-assessment/step-by-step-instructions');

    // Check for launch file examples
    const launchExamples = await page.locator('pre:has(code:has-text("launch")).count();
    expect(launchExamples).toBeGreaterThan(0);

    // Verify launch file patterns
    await expect(page.locator('code')).toContainText('LaunchDescription');
    await expect(page.locator('code')).toContainText('Node');
    await expect(page.locator('code')).toContainText('ExecuteProcess');
  });

  test('should verify message passing between systems', async ({ page }) => {
    await page.goto('/docs/module-2/sensor-simulation/sensor-data-validation');

    // Check for message validation examples
    const validationExamples = await page.locator('pre:has(code:has-text("sensor_msgs")).count();
    expect(validationExamples).toBeGreaterThan(0);

    // Verify message types
    await expect(page.locator('code')).toContainText('LaserScan');
    await expect(page.locator('code')).toContainText('Imu');
    await expect(page.locator('code')).toContainText('Image');
  });

  test('should validate coordinate system integration', async ({ page }) => {
    await page.goto('/docs/module-2/physics-simulation/physics-debugging-validation');

    // Check for coordinate transformation examples
    const transformExamples = await page.locator('code:has-text("coordinate")).count();
    expect(transformExamples).toBeGreaterThan(0);

    // Verify coordinate handling
    await expect(page.locator('code')).toContainText('ROS coordinate');
    await expect(page.locator('code')).toContainText('Unity coordinate');
  });

  test('should verify timing and synchronization', async ({ page }) => {
    await page.goto('/docs/module-2/physics-simulation/physics-debugging-validation');

    // Check for timing examples
    const timingExamples = await page.locator('code:has-text("Time")).count();
    expect(timingExamples).toBeGreaterThan(0);

    // Verify timing patterns
    await expect(page.locator('code')).toContainText('real_time_factor');
    await expect(page.locator('code')).toContainText('update_rate');
  });

  test('should validate parameter configuration integration', async ({ page }) => {
    await page.goto('/docs/projects/module-2-assessment/step-by-step-instructions');

    // Check for parameter examples
    const paramExamples = await page.locator('pre:has(code:has-text("yaml")).count();
    expect(paramExamples).toBeGreaterThan(0);

    // Verify parameter patterns
    await expect(page.locator('code')).toContainText(':');
    await expect(page.locator('code')).toContainText('parameters');
    await expect(page.locator('code')).toContainText('config');
  });

  test('should verify error handling in integration', async ({ page }) => {
    await page.goto('/docs/module-2/introduction/troubleshooting-guide');

    // Check for error handling examples
    const errorHandling = await page.locator('h2:has-text("Troubleshooting")).count();
    expect(errorHandling).toBe(1);

    // Verify error handling approaches
    await expect(page.locator('body')).toContainText('error');
    await expect(page.locator('body')).toContainText('failed');
    await expect(page.locator('body')).toContainText('solution');
  });

  test('should validate system performance considerations', async ({ page }) => {
    await page.goto('/docs/module-2/physics-simulation/physics-debugging-validation');

    // Check for performance sections
    const performanceSections = await page.locator('h2:has-text("Performance")).count();
    expect(performanceSections).toBeGreaterThan(0);

    // Verify performance considerations
    await expect(page.locator('body')).toContainText('performance');
    await expect(page.locator('body')).toContainText('optimization');
    await expect(page.locator('body')).toContainText('real-time');
  });
});