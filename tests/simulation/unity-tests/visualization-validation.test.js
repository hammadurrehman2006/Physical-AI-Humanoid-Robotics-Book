/**
 * Unity Visualization Validation Tests
 * Validates Unity integration examples work with ROS 2 bridge
 */

const { test, expect } = require('@playwright/test');

test.describe('Unity Visualization Validation', () => {
  test('should validate Unity setup documentation', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/unity-setup');

    // Verify Unity setup documentation contains essential information
    await expect(page.locator('h1')).toContainText('Unity Setup');

    // Check for installation steps
    const installationSteps = await page.locator('h2:has-text("Unity Installation")').count();
    expect(installationSteps).toBe(1);

    // Verify Unity Hub installation is covered
    await expect(page.locator('body')).toContainText('Unity Hub');
    await expect(page.locator('body')).toContainText('LTS');
    await expect(page.locator('body')).toContainText('2022.3');
  });

  test('should validate ROS 2 Unity bridge documentation', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/ros2-unity-bridge');

    // Verify bridge documentation contains essential elements
    await expect(page.locator('h1')).toContainText('ROS 2 to Unity Bridge');

    // Check for architecture section
    const architectureSection = await page.locator('h2:has-text("Bridge Architecture")').count();
    expect(architectureSection).toBe(1);

    // Verify implementation steps are documented
    const implementationSteps = await page.locator('h2:has-text("Implementation Steps")').count();
    expect(implementationSteps).toBe(1);

    // Check for code examples
    const codeExamples = await page.locator('pre:has(code:has-text("ROSConnection"))').count();
    expect(codeExamples).toBeGreaterThan(0);

    // Verify Unity-specific ROS patterns
    await expect(page.locator('code')).toContainText('Unity.Robotics.ROSTCPConnector');
    await expect(page.locator('code')).toContainText('RosMessageTypes');
  });

  test('should validate visualization techniques documentation', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/visualization-techniques');

    // Verify visualization documentation contains essential elements
    await expect(page.locator('h1')).toContainText('Visualization Techniques');

    // Check for core concepts
    const coreConcepts = await page.locator('h2:has-text("Core Visualization Concepts")').count();
    expect(coreConcepts).toBe(1);

    // Verify lighting techniques are covered
    await expect(page.locator('body')).toContainText('lighting');
    await expect(page.locator('body')).toContainText('shadows');
    await expect(page.locator('body')).toContainText('material');

    // Check for code examples for visualization
    const visualizationCode = await page.locator('pre:has(code:has-text("JointVisualizer"))').count();
    expect(visualizationCode).toBeGreaterThan(0);
  });

  test('should validate Unity troubleshooting documentation', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/unity-troubleshooting');

    // Verify troubleshooting documentation structure
    await expect(page.locator('h1')).toContainText('Unity Troubleshooting');

    // Check for common issues section
    const commonIssues = await page.locator('h2:has-text("Common Unity-ROS 2 Bridge Issues")').count();
    expect(commonIssues).toBe(1);

    // Verify troubleshooting approaches
    await expect(page.locator('body')).toContainText('troubleshooting');
    await expect(page.locator('body')).toContainText('connection');
    await expect(page.locator('body')).toContainText('network');
  });

  test('should verify Unity-ROS 2 communication patterns', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/ros2-unity-bridge');

    // Check for publisher examples
    const publisherExamples = await page.locator('code:has-text("Publish")').count();
    expect(publisherExamples).toBeGreaterThan(0);

    // Check for subscriber examples
    const subscriberExamples = await page.locator('code:has-text("Subscribe")').count();
    expect(subscriberExamples).toBeGreaterThan(0);

    // Verify common ROS message types are shown
    await expect(page.locator('code')).toContainText('TwistMsg');
    await expect(page.locator('code')).toContainText('JointStateMsg');
    await expect(page.locator('code')).toContainText('LaserScanMsg');
  });

  test('should validate performance optimization techniques', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/visualization-techniques');

    // Check for performance section
    const performanceSection = await page.locator('h2:has-text("Performance Optimization")').count();
    expect(performanceSection).toBe(1);

    // Verify performance techniques are documented
    await expect(page.locator('body')).toContainText('LOD');
    await expect(page.locator('body')).toContainText('occlusion');
    await expect(page.locator('body')).toContainText('framerate');
  });

  test('should verify sensor data visualization examples', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/visualization-techniques');

    // Check for sensor visualization code
    const lidarVizCode = await page.locator('pre:has(code:has-text("LidarVisualizer"))').count();
    expect(lidarVizCode).toBeGreaterThan(0);

    const cameraVizCode = await page.locator('pre:has(code:has-text("CameraFeedVisualizer"))').count();
    expect(cameraVizCode).toBeGreaterThan(0);

    // Verify sensor data handling
    await expect(page.locator('code')).toContainText('ranges');
    await expect(page.locator('code')).toContainText('imageData');
    await expect(page.locator('code')).toContainText('pointCloud');
  });

  test('should validate coordinate system transformation examples', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/visualization-techniques');

    // Check for coordinate transformation examples
    const transformCode = await page.locator('code:has-text("Quaternion")).count();
    expect(transformCode).toBeGreaterThan(0);

    // Verify coordinate system handling
    await expect(page.locator('code')).toContainText('RosToUnityPosition');
    await expect(page.locator('code')).toContainText('RosToUnityRotation');
  });

  test('should verify Unity project structure documentation', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/unity-setup');

    // Check for project configuration guidance
    const projectConfig = await page.locator('h3:has-text("Project Configuration")').count();
    expect(projectConfig).toBeGreaterThan(0);

    // Verify essential Unity settings are covered
    await expect(page.locator('body')).toContainText('Player Settings');
    await expect(page.locator('body')).toContainText('Package Manager');
    await expect(page.locator('body')).toContainText('Scene Structure');
  });

  test('should validate Unity testing and validation approaches', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/ros2-unity-bridge');

    // Check for connection testing examples
    const testCode = await page.locator('pre:has(code:has-text("BridgeTester"))').count();
    expect(testCode).toBeGreaterThan(0);

    // Verify testing patterns
    await expect(page.locator('code')).toContainText('IsConnected');
    await expect(page.locator('code')).toContainText('TestConnection');
    await expect(page.locator('code')).toContainText('lastMessageTime');
  });

  test('should verify Unity asset integration patterns', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/visualization-techniques');

    // Check for asset integration examples
    const assetCode = await page.locator('code:has-text("RobotMaterialController")).count();
    expect(assetCode).toBeGreaterThan(0);

    // Verify material and asset handling
    await expect(page.locator('code')).toContainText('Material');
    await expect(page.locator('code')).toContainText('Renderer');
    await expect(page.locator('code')).toContainText('Transform');
  });

  test('should validate security and network considerations', async ({ page }) => {
    await page.goto('/docs/module-2/unity-integration/ros2-unity-bridge');

    // Check for security section
    const securitySection = await page.locator('h2:has-text("Security Considerations")').count();
    expect(securitySection).toBe(1);

    // Verify security topics are covered
    await expect(page.locator('body')).toContainText('security');
    await expect(page.locator('body')).toContainText('authentication');
    await expect(page.locator('body')).toContainText('firewall');
  });
});