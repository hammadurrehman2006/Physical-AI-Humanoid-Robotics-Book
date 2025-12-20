/**
 * Link Validation Tests for Module 2 Documentation
 * Validates that all navigation links throughout Module 2 work correctly
 */

const { test, expect } = require('@playwright/test');

test.describe('Module 2 Link Validation', () => {
  test('should navigate through all Module 2 pages without broken links', async ({ page }) => {
    // Start from the Module 2 index page
    await page.goto('/docs/module-2');

    // Verify the page loads correctly
    await expect(page).toHaveTitle(/Module 2/);
    await expect(page.locator('h1')).toContainText('Module 2');

    // Test navigation to introduction section
    await page.click('text=Introduction');
    await expect(page).toHaveURL(/introduction/);

    // Test prerequisites page
    await page.click('text=Prerequisites');
    await expect(page).toHaveURL(/prerequisites/);

    // Test setup page
    await page.click('text=Setting up Gazebo Environment');
    await expect(page).toHaveURL(/setup-gazebo-environment/);

    // Test basic robot spawning
    await page.click('text=Basic Robot Spawning');
    await expect(page).toHaveURL(/basic-robot-spawning/);

    // Test troubleshooting
    await page.click('text=Troubleshooting');
    await expect(page).toHaveURL(/troubleshooting/);
  });

  test('should validate URDF/SDF format navigation', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Navigate to URDF/SDF formats section
    await page.click('text=URDF/SDF Formats');
    await expect(page.locator('h1')).toContainText('URDF');

    // Test all URDF/SDF pages
    const urdfSdfPages = [
      'URDF Basics',
      'SDF Advanced',
      'Creating Robot Models',
      'Conversion Guide',
      'Practical URDF/SDF Examples'
    ];

    for (const pageTitle of urdfSdfPages) {
      await page.click(`text=${pageTitle}`);
      await expect(page).toHaveURL(/urdf-sdf-formats/);
      await page.goBack();
    }
  });

  test('should validate physics simulation navigation', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Navigate to physics simulation section
    await page.click('text=Physics Simulation');

    // Test all physics pages
    const physicsPages = [
      'Gravity and Collisions',
      'Material Properties',
      'Environment Modeling',
      'Physics Debugging and Validation'
    ];

    for (const pageTitle of physicsPages) {
      await page.click(`text=${pageTitle}`);
      await expect(page).toHaveURL(/physics-simulation/);
      await page.goBack();
    }
  });

  test('should validate sensor simulation navigation', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Navigate to sensor simulation section
    await page.click('text=Sensor Simulation');

    // Test all sensor pages
    const sensorPages = [
      'LiDAR Simulation',
      'Camera Simulation',
      'IMU Simulation',
      'Sensor Fusion',
      'Sensor Data Validation'
    ];

    for (const pageTitle of sensorPages) {
      await page.click(`text=${pageTitle}`);
      await expect(page).toHaveURL(/sensor-simulation/);
      await page.goBack();
    }
  });

  test('should validate Unity integration navigation', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Navigate to Unity integration section
    await page.click('text=Unity Integration');

    // Test all Unity pages
    const unityPages = [
      'Unity Setup',
      'ROS 2 Unity Bridge',
      'Visualization Techniques',
      'Unity Troubleshooting'
    ];

    for (const pageTitle of unityPages) {
      await page.click(`text=${pageTitle}`);
      await expect(page).toHaveURL(/unity-integration/);
      await page.goBack();
    }
  });

  test('should validate assessment project navigation', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Navigate to assessment project section
    await page.click('text=Assessment Project');

    // Test all assessment pages
    const assessmentPages = [
      'Project Overview',
      'Requirements',
      'Evaluation Criteria'
    ];

    for (const pageTitle of assessmentPages) {
      await page.click(`text=${pageTitle}`);
      await expect(page).toHaveURL(/assessment-project/);
      await page.goBack();
    }
  });

  test('should validate assessment project detailed navigation', async ({ page }) => {
    await page.goto('/docs/projects/module-2-assessment');

    // Test assessment project detailed pages
    const projectPages = [
      'Project Scaffolding',
      'Step-by-Step Instructions',
      'Solution Examples'
    ];

    for (const pageTitle of projectPages) {
      await page.click(`text=${pageTitle}`);
      await expect(page).toHaveURL(/module-2-assessment/);
      await page.goBack();
    }
  });

  test('should check for 404 errors on all Module 2 links', async ({ page }) => {
    const response = await page.goto('/docs/module-2');
    expect(response.status()).toBe(200);

    // Get all links on the Module 2 page
    const links = await page.locator('a[href^="/docs/module-2"]').all();

    for (const link of links) {
      const href = await link.getAttribute('href');
      if (href && !href.includes('http')) { // Skip external links
        const linkResponse = await page.request.get(`http://localhost:3000${href}`);
        expect(linkResponse.status()).toBe(200);
      }
    }
  });
});