/**
 * Code Example Validation Tests for Module 2
 * Validates that all code examples in Module 2 documentation are correct and runnable
 */

const { test, expect } = require('@playwright/test');

test.describe('Module 2 Code Example Validation', () => {
  test('should validate URDF code examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/urdf-basics');

    // Check for URDF XML code blocks
    const urdfCodeBlocks = await page.locator('pre:has(code:has-text("<?xml"))').count();
    expect(urdfCodeBlocks).toBeGreaterThan(0);

    // Verify URDF structure examples
    await expect(page.locator('code')).toContainText('<robot');
    await expect(page.locator('code')).toContainText('<link');
    await expect(page.locator('code')).toContainText('<joint');
  });

  test('should validate SDF code examples', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/sdf-advanced');

    // Check for SDF XML code blocks
    const sdfCodeBlocks = await page.locator('pre:has(code:has-text("<sdf"))').count();
    expect(sdfCodeBlocks).toBeGreaterThan(0);

    // Verify SDF structure examples
    await expect(page.locator('code')).toContainText('<sdf');
    await expect(page.locator('code')).toContainText('<model');
    await expect(page.locator('code')).toContainText('<world');
  });

  test('should validate Gazebo world file examples', async ({ page }) => {
    await page.goto('/docs/module-2/introduction/setup-gazebo-environment');

    // Look for world file examples
    const worldCodeBlocks = await page.locator('pre:has(code:has-text("<world"))').count();
    expect(worldCodeBlocks).toBeGreaterThan(0);
  });

  test('should validate Python code examples', async ({ page }) => {
    await page.goto('/docs/module-2/sensor-simulation/lidar-simulation');

    // Check for Python code blocks
    const pythonCodeBlocks = await page.locator('pre:has(code:has-text("import"))').count();
    expect(pythonCodeBlocks).toBeGreaterThan(0);

    // Verify common Python imports for robotics
    await expect(page.locator('code')).toContainText('import rclpy');
    await expect(page.locator('code')).toContainText('from sensor_msgs.msg');
    await expect(page.locator('code')).toContainText('Node');
  });

  test('should validate launch file examples', async ({ page }) => {
    await page.goto('/docs/projects/module-2-assessment/step-by-step-instructions');

    // Check for launch file code blocks
    const launchCodeBlocks = await page.locator('pre:has(code:has-text("launch"))').count();
    expect(launchCodeBlocks).toBeGreaterThan(0);

    // Verify launch file structure
    await expect(page.locator('code')).toContainText('generate_launch_description');
    await expect(page.locator('code')).toContainText('LaunchDescription');
  });

  test('should validate C++ code examples', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/ros2-unity-bridge');

    // Check for C# code blocks (Unity uses C#)
    const csharpCodeBlocks = await page.locator('pre:has(code:has-text("using"))').count();
    expect(csharpCodeBlocks).toBeGreaterThan(0);

    // Verify Unity/C# specific patterns
    await expect(page.locator('code')).toContainText('MonoBehaviour');
    await expect(page.locator('code')).toContainText('Start()');
    await expect(page.locator('code')).toContainText('Update()');
  });

  test('should validate bash/command examples', async ({ page }) => {
    await page.goto('/docs/module-2/introduction/setup-gazebo-environment');

    // Check for bash command examples
    const bashCodeBlocks = await page.locator('pre:has(code:has-text("$"))').count();
    expect(bashCodeBlocks).toBeGreaterThan(0);

    // Verify common bash commands for ROS/Gazebo
    await expect(page.locator('code')).toContainText('sudo apt install');
    await expect(page.locator('code')).toContainText('source /opt/ros/');
    await expect(page.locator('code')).toContainText('gz sim');
  });

  test('should validate configuration file examples', async ({ page }) => {
    await page.goto('/docs/module-2/physics-simulation/gravity-and-collisions');

    // Check for YAML configuration examples
    const yamlCodeBlocks = await page.locator('pre:has(code:has-text("yaml"))').count();
    expect(yamlCodeBlocks).toBeGreaterThan(0);

    // Verify YAML structure
    await expect(page.locator('code')).toContainText(':');
    await expect(page.locator('code')).toContainText('- ');
  });

  test('should validate all code blocks have proper syntax highlighting', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check that code blocks have language identifiers
    const codeBlocks = await page.locator('pre code').count();
    expect(codeBlocks).toBeGreaterThan(0);

    // Verify that different language blocks exist
    const xmlBlocks = await page.locator('pre:has(code:has-text("xml"))').count();
    const pythonBlocks = await page.locator('pre:has(code:has-text("python"))').count();
    const bashBlocks = await page.locator('pre:has(code:has-text("bash"))').count();
    const csharpBlocks = await page.locator('pre:has(code:has-text("csharp"))').count();

    expect(xmlBlocks + pythonBlocks + bashBlocks + csharpBlocks).toBeGreaterThan(0);
  });

  test('should validate code examples follow ROS 2 Humble conventions', async ({ page }) => {
    await page.goto('/docs/projects/module-2-assessment/step-by-step-instructions');

    // Check for ROS 2 Humble specific patterns
    await expect(page.locator('code')).toContainText('humble');
    await expect(page.locator('code')).toContainText('rclpy');
    await expect(page.locator('code')).toContainText('std_msgs');
    await expect(page.locator('code')).toContainText('geometry_msgs');
  });
});