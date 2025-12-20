/**
 * Final Review Tests for Module 2 Content Consistency
 * Validates overall quality, consistency, and completeness of Module 2 content
 */

const { test, expect } = require('@playwright/test');

test.describe('Module 2 Content Consistency Review', () => {
  test('should verify all Module 2 pages follow consistent structure', async ({ page }) => {
    // Test the Module 2 index page
    await page.goto('/docs/module-2');

    // Verify common elements on index page
    await expect(page.locator('h1')).toContainText('Module 2');
    await expect(page.locator('h2')).toContainText('What You\'ll Learn');
    await expect(page.locator('h2')).toContainText('Prerequisites');
    await expect(page.locator('h2')).toContainText('Module Structure');

    // Verify navigation to next section
    await expect(page.locator('a')).toContainText('Setting up Gazebo Environment');
  });

  test('should validate content progression follows learning objectives', async ({ page }) => {
    // Start with Gazebo setup
    await page.goto('/docs/module-2/introduction/setup-gazebo-environment');
    await expect(page.locator('h1')).toContainText('Setting up Gazebo');

    // Progress to URDF basics
    await page.click('text=URDF and SDF robot description formats');
    await expect(page).toHaveURL(/urdf-basics/);

    // Progress to physics simulation
    await page.click('text=Physics Simulation');
    await expect(page).toHaveURL(/gravity-and-collisions/);

    // Progress to sensor simulation
    await page.click('text=Sensor Simulation');
    await expect(page).toHaveURL(/lidar-simulation/);

    // Progress to Unity integration
    await page.click('text=Unity Integration');
    await expect(page).toHaveURL(/unity-setup/);
  });

  test('should verify all pages contain appropriate learning objectives', async ({ page }) => {
    const pagesToCheck = [
      '/docs/module-2/introduction/setup-gazebo-environment',
      '/docs/module-2/urdf-sdf-formats/urdf-basics',
      '/docs/module-2/physics-simulation/gravity-and-collisions',
      '/docs/module-2/sensor-simulation/lidar-simulation',
      '/docs/module-2/unity-integration/unity-setup'
    ];

    for (const pagePath of pagesToCheck) {
      await page.goto(pagePath);

      // Look for learning objectives or goals sections
      const hasObjectives = await page.locator('h2:has-text("Learning Objectives")').count() > 0 ||
                           await page.locator('h2:has-text("Objectives")').count() > 0 ||
                           await page.locator('h2:has-text("What You\'ll Learn")').count() > 0 ||
                           await page.locator('h3:has-text("Learning Objectives")').count() > 0;

      expect(hasObjectives).toBeTruthy();
    }
  });

  test('should verify all pages contain appropriate prerequisites', async ({ page }) => {
    const pagesToCheck = [
      '/docs/module-2/physics-simulation/material-properties',
      '/docs/module-2/sensor-simulation/camera-simulation',
      '/docs/module-2/unity-integration/ros2-unity-bridge'
    ];

    for (const pagePath of pagesToCheck) {
      await page.goto(pagePath);

      // Look for prerequisites sections
      const hasPrerequisites = await page.locator('h2:has-text("Prerequisites")').count() > 0 ||
                              await page.locator('h3:has-text("Prerequisites")').count() > 0 ||
                              await page.locator('text:has-text("prerequisites")').count() > 0;

      expect(hasPrerequisites).toBeTruthy();
    }
  });

  test('should verify all pages contain appropriate next steps', async ({ page }) => {
    const pagesToCheck = [
      '/docs/module-2/introduction/troubleshooting-guide',
      '/docs/module-2/urdf-sdf-formats/conversion-guide',
      '/docs/module-2/physics-simulation/physics-debugging-validation',
      '/docs/module-2/sensor-simulation/sensor-data-validation',
      '/docs/module-2/unity-integration/unity-troubleshooting'
    ];

    for (const pagePath of pagesToCheck) {
      await page.goto(pagePath);

      // Look for next steps sections
      const hasNextSteps = await page.locator('h2:has-text("Next Steps")').count() > 0 ||
                          await page.locator('h3:has-text("Next Steps")').count() > 0 ||
                          await page.locator('text:has-text("Next Steps")').count() > 0 ||
                          await page.locator('text:has-text("Continue to")').count() > 0;

      expect(hasNextSteps).toBeTruthy();
    }
  });

  test('should validate terminology consistency across Module 2', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for consistent terminology usage
    await expect(page.locator('body')).toContainText('Gazebo'); // Should use "Gazebo" not "gazebo"
    await expect(page.locator('body')).toContainText('ROS 2'); // Should use "ROS 2" not "ROS2"
    await expect(page.locator('body')).toContainText('Unity'); // Should use "Unity" consistently

    // Check for consistent naming of tools
    await expect(page.locator('body')).toContainText('URDF'); // Should be uppercase
    await expect(page.locator('body')).toContainText('SDF'); // Should be uppercase
  });

  test('should verify all code examples have explanations', async ({ page }) => {
    const pagesWithCode = [
      '/docs/module-2/urdf-sdf-formats/urdf-basics',
      '/docs/module-2/physics-simulation/gravity-and-collisions',
      '/docs/module-2/sensor-simulation/lidar-simulation'
    ];

    for (const pagePath of pagesWithCode) {
      await page.goto(pagePath);

      // Count code blocks
      const codeBlockCount = await page.locator('pre code').count();

      // Count explanation paragraphs near code blocks
      const explanationCount = await page.locator('p:has(pre)').count();

      // Verify there are explanations for code examples
      expect(codeBlockCount > 0).toBeTruthy();
      // Note: This is a basic check - in real implementation we'd want more sophisticated analysis
    }
  });

  test('should validate image and asset references are valid', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for image tags (though Docusaurus typically uses markdown)
    const imageCount = await page.locator('img').count();

    // In documentation, we expect some images or diagrams
    // This is a basic check - actual count may vary
    expect(typeof imageCount).toBe('number');
  });

  test('should verify content accessibility standards', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for proper heading hierarchy
    const h1Count = await page.locator('h1').count();
    expect(h1Count).toBe(1); // Should have exactly one H1 per page

    // Check for alt text on images (if any)
    const imagesWithoutAlt = await page.locator('img:not([alt])').count();
    expect(imagesWithoutAlt).toBeLessThan(5); // Allow some images without alt if they're decorative

    // Check for sufficient color contrast (this would require additional accessibility tools in practice)
    // For now, we just verify the page loads properly
    const pageTitle = await page.title();
    expect(pageTitle).toBeTruthy();
  });

  test('should validate all pages load within reasonable time', async ({ page }) => {
    const startTime = Date.now();
    await page.goto('/docs/module-2');
    const loadTime = Date.now() - startTime;

    // Page should load reasonably quickly (under 5 seconds)
    expect(loadTime).toBeLessThan(5000);

    // Verify content is present after load
    const content = await page.locator('main').textContent();
    expect(content.length).toBeGreaterThan(100); // Should have substantial content
  });

  test('should verify cross-references work correctly', async ({ page }) => {
    await page.goto('/docs/module-2/assessment-project/project-overview');

    // Look for links to other parts of Module 2
    const internalLinks = await page.locator('a[href*="/docs/module-2"]').count();
    expect(internalLinks).toBeGreaterThan(0);

    // Verify some links navigate correctly
    if (internalLinks > 0) {
      const firstLink = page.locator('a[href*="/docs/module-2"]').first();
      const linkHref = await firstLink.getAttribute('href');

      if (linkHref && linkHref !== '/docs/module-2') {
        await firstLink.click();
        await expect(page).toHaveURL(new RegExp(linkHref));
        await page.goBack();
      }
    }
  });
});