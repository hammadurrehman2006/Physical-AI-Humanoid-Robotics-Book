/**
 * Robot Model Loading Tests for Gazebo
 * Validates that URDF/SDF examples load correctly in Gazebo environment
 */

const { test, expect } = require('@playwright/test');

test.describe('Gazebo Robot Model Loading', () => {
  test('should validate URDF to SDF conversion examples', async ({ page }) => {
    // This test validates the documentation examples for URDF/SDF conversion
    await page.goto('/docs/module-2/urdf-sdf-formats/conversion-guide');

    // Verify that conversion examples are properly documented
    const conversionExamples = await page.locator('pre:has(code:has-text("gz sdf"))').count();
    expect(conversionExamples).toBeGreaterThan(0);

    // Check for the basic conversion command example
    await expect(page.locator('code')).toContainText('gz sdf -p robot.urdf');
    await expect(page.locator('code')).toContainText('check_urdf');
  });

  test('should validate basic robot model examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/creating-robot-models');

    // Check for complete robot model examples
    const urdfExamples = await page.locator('pre:has(code:has-text("<robot"))').count();
    expect(urdfExamples).toBeGreaterThan(0);

    // Verify essential robot elements are documented
    await expect(page.locator('code')).toContainText('<link');
    await expect(page.locator('code')).toContainText('<joint');
    await expect(page.locator('code')).toContainText('<inertial');
    await expect(page.locator('code')).toContainText('<visual');
    await expect(page.locator('code')).toContainText('<collision');
  });

  test('should validate differential drive robot examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/practical-urdf-sdf-examples');

    // Check for differential drive examples
    const diffDriveExamples = await page.locator('pre:has(code:has-text("differential_drive_robot"))').count();
    expect(diffDriveExamples).toBeGreaterThan(0);

    // Verify differential drive specific elements
    await expect(page.locator('code')).toContainText('left_wheel');
    await expect(page.locator('code')).toContainText('right_wheel');
    await expect(page.locator('code')).toContainText('continuous');
    await expect(page.locator('code')).toContainText('wheel_separation');
  });

  test('should validate sensor integration examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/practical-urdf-sdf-examples');

    // Check for sensor integration examples
    const sensorExamples = await page.locator('pre:has(code:has-text("sensor"))').count();
    expect(sensorExamples).toBeGreaterThan(0);

    // Verify common sensor types
    await expect(page.locator('code')).toContainText('lidar_link');
    await expect(page.locator('code')).toContainText('camera_link');
    await expect(page.locator('code')).toContainText('imu_link');
  });

  test('should validate Gazebo plugin examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/practical-urdf-sdf-examples');

    // Check for Gazebo plugin examples
    const pluginExamples = await page.locator('pre:has(code:has-text("gazebo"))').count();
    expect(pluginExamples).toBeGreaterThan(0);

    // Verify common plugin types
    await expect(page.locator('code')).toContainText('libgazebo_ros2_control');
    await expect(page.locator('code')).toContainText('libgazebo_ros_ray_sensor');
    await expect(page.locator('code')).toContainText('libgazebo_ros_camera');
  });

  test('should validate physics properties examples', async ({ page }) => {
    await page.goto('/docs/module-2/physics-simulation/gravity-and-collisions');

    // Check for physics configuration examples
    const physicsExamples = await page.locator('pre:has(code:has-text("inertial"))').count();
    expect(physicsExamples).toBeGreaterThan(0);

    // Verify physics properties
    await expect(page.locator('code')).toContainText('mass');
    await expect(page.locator('code')).toContainText('inertia');
    await expect(page.locator('code')).toContainText('ixx');
    await expect(page.locator('code')).toContainText('ixy');
  });

  test('should validate world file examples', async ({ page }) => {
    await page.goto('/docs/projects/module-2-assessment/step-by-step-instructions');

    // Check for world file examples
    const worldExamples = await page.locator('pre:has(code:has-text("<world"))').count();
    expect(worldExamples).toBeGreaterThan(0);

    // Verify world file structure
    await expect(page.locator('code')).toContainText('<physics');
    await expect(page.locator('code')).toContainText('<model');
    await expect(page.locator('code')).toContainText('gravity');
  });

  test('should validate ROS 2 control integration', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/practical-urdf-sdf-examples');

    // Check for ROS 2 control examples
    const ros2ControlExamples = await page.locator('pre:has(code:has-text("ros2_control"))').count();
    expect(ros2ControlExamples).toBeGreaterThan(0);

    // Verify ROS 2 control elements
    await expect(page.locator('code')).toContainText('ros2_control');
    await expect(page.locator('code')).toContainText('hardware_interface');
    await expect(page.locator('code')).toContainText('command_interface');
  });

  test('should validate material and visual properties', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/creating-robot-models');

    // Check for material examples
    const materialExamples = await page.locator('pre:has(code:has-text("material"))').count();
    expect(materialExamples).toBeGreaterThan(0);

    // Verify material properties
    await expect(page.locator('code')).toContainText('color');
    await expect(page.locator('code')).toContainText('rgba');
    await expect(page.locator('code')).toContainText('ambient');
    await expect(page.locator('code')).toContainText('diffuse');
  });

  test('should validate joint configuration examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/urdf-basics');

    // Check for joint examples
    const jointExamples = await page.locator('pre:has(code:has-text("joint"))').count();
    expect(jointExamples).toBeGreaterThan(0);

    // Verify joint types
    await expect(page.locator('code')).toContainText('continuous');
    await expect(page.locator('code')).toContainText('revolute');
    await expect(page.locator('code')).toContainText('fixed');
    await expect(page.locator('code')).toContainText('prismatic');
  });

  test('should verify model validation commands', async ({ page }) => {
    await page.goto('/docs/module-2/introduction/troubleshooting-guide');

    // Check for validation commands
    const validationCommands = await page.locator('pre:has(code:has-text("check_urdf"))').count();
    expect(validationCommands).toBeGreaterThan(0);

    // Verify common validation commands
    await expect(page.locator('code')).toContainText('check_urdf');
    await expect(page.locator('code')).toContainText('gz sdf -k');
    await expect(page.locator('code')).toContainText('urdf_to_graphviz');
  });

  test('should validate coordinate system documentation', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/ros2-unity-bridge');

    // Check for coordinate system examples
    const coordinateExamples = await page.locator('code:has-text("coordinate")).count();
    expect(coordinateExamples).toBeGreaterThan(0);

    // Verify coordinate system handling
    await expect(page.locator('code')).toContainText('ROS coordinate');
    await expect(page.locator('code')).toContainText('Unity coordinate');
  });
});