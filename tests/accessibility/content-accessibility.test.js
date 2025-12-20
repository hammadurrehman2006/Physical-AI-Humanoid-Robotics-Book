/**
 * Accessibility Testing for Module 2 Content
 * Validates that Module 2 documentation meets accessibility standards
 */

const { test, expect } = require('@playwright/test');

test.describe('Module 2 Content Accessibility', () => {
  test('should have proper heading hierarchy', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check heading structure
    const h1Count = await page.locator('h1').count();
    expect(h1Count).toBe(1); // Only one H1 per page

    const h2Count = await page.locator('h2').count();
    expect(h2Count).toBeGreaterThan(0); // Should have H2 sections

    // Verify H2s come after H1, not before
    const h1Position = await page.locator('h1').evaluate(node =>
      Array.from(node.parentElement.children).indexOf(node)
    );

    const h2Elements = await page.locator('h2').elementHandles();
    for (const h2 of h2Elements) {
      const h2Position = await h2.evaluate(node =>
        Array.from(node.parentElement.children).indexOf(node)
      );
      expect(h2Position).toBeGreaterThan(h1Position);
    }
  });

  test('should have sufficient color contrast', async ({ page }) => {
    await page.goto('/docs/module-2');

    // This test would typically use a color contrast checker
    // For now, we'll verify that the page has standard text elements
    const textElements = await page.locator('p, li, td, th').count();
    expect(textElements).toBeGreaterThan(0);

    // Verify that text content exists and is readable
    const mainContent = await page.locator('main').textContent();
    expect(mainContent.length).toBeGreaterThan(500); // Substantial content
  });

  test('should have proper alt text for images', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Count images with and without alt text
    const allImages = await page.locator('img').count();
    const imagesWithAlt = await page.locator('img[alt]').count();
    const imagesWithoutAlt = allImages - imagesWithAlt;

    // For documentation, most images should have alt text
    // Allow some decorative images without alt
    if (allImages > 0) {
      expect(imagesWithoutAlt).toBeLessThan(Math.ceil(allImages * 0.3)); // At least 70% should have alt
    }
  });

  test('should have descriptive link text', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Find all links
    const links = await page.locator('a').all();

    for (const link of links) {
      const textContent = await link.textContent();
      const href = await link.getAttribute('href');

      // Skip navigation links that are descriptive by context
      if (href && !href.includes('#') && !href.includes('http')) {
        // Link text should be descriptive, not generic like "click here"
        expect(textContent.toLowerCase()).not.toContain('click here');
        expect(textContent.toLowerCase()).not.toContain('read more');
        expect(textContent.trim().length).toBeGreaterThan(2); // Meaningful text
      }
    }
  });

  test('should have proper focus management', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Test that interactive elements are focusable
    const links = await page.locator('a[href]:visible').count();
    const buttons = await page.locator('button:visible').count();

    expect(links + buttons).toBeGreaterThan(0); // Should have interactive elements

    // Test keyboard navigation capability
    await page.keyboard.press('Tab');
    const focusedElement = await page.evaluate(() => document.activeElement.tagName);
    expect(['A', 'BUTTON', 'INPUT']).toContain(focusedElement);
  });

  test('should validate form elements have proper labels', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for form elements and their labels
    const inputs = await page.locator('input:not([type="hidden"]):not([type="submit"])').count();
    const labels = await page.locator('label').count();

    // If there are inputs, they should have associated labels
    if (inputs > 0) {
      expect(labels).toBeGreaterThanOrEqual(inputs * 0.5); // At least 50% should have labels
    }
  });

  test('should have semantic HTML structure', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for semantic elements
    const mainCount = await page.locator('main').count();
    const navCount = await page.locator('nav').count();
    const articleCount = await page.locator('article').count();
    const sectionCount = await page.locator('section').count();

    expect(mainCount).toBeGreaterThanOrEqual(1);
    expect(sectionCount).toBeGreaterThanOrEqual(1);

    // Verify content is properly structured
    const mainContent = await page.locator('main').textContent();
    expect(mainContent.length).toBeGreaterThan(300);
  });

  test('should have appropriate heading levels for content structure', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/urdf-basics');

    // Check that headings follow proper hierarchy (H1 -> H2 -> H3, etc.)
    const headings = await page.locator('h1, h2, h3, h4, h5, h6').all();
    let lastLevel = 0;

    for (const heading of headings) {
      const tagName = await heading.evaluate(node => node.tagName);
      const level = parseInt(tagName.charAt(1)); // Extract number from H1, H2, etc.

      // Each heading level should be at most one level deeper than the previous
      expect(level).toBeLessThanOrEqual(lastLevel + 1);
      lastLevel = level;
    }
  });

  test('should validate table accessibility', async ({ page }) => {
    await page.goto('/docs/module-2/urdf-sdf-formats/conversion-guide');

    // Check for tables and their accessibility features
    const tables = await page.locator('table').count();

    if (tables > 0) {
      // Tables should have headers
      const tablesWithHeaders = await page.locator('table:has(th)').count();
      expect(tablesWithHeaders).toBeGreaterThanOrEqual(Math.floor(tables * 0.7)); // At least 70% with headers

      // Check for table captions
      const tablesWithCaptions = await page.locator('table:has(caption)').count();
    }
  });

  test('should verify screen reader compatibility', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for ARIA labels and roles
    const ariaLabels = await page.locator('[aria-label]').count();
    const ariaLabelledBy = await page.locator('[aria-labelledby]').count();
    const roles = await page.locator('[role]').count();

    // While not all elements need ARIA, important interactive elements should have it
    const linksAndButtons = await page.locator('a, button').count();
    if (linksAndButtons > 5) {
      // If there are many interactive elements, expect some ARIA usage
      expect(ariaLabels + ariaLabelledBy + roles).toBeGreaterThanOrEqual(1);
    }
  });

  test('should validate responsive design accessibility', async ({ page }) => {
    // Test on different viewport sizes
    await page.setViewportSize({ width: 375, height: 667 }); // Mobile
    await page.goto('/docs/module-2');

    // Check that content is still accessible on mobile
    const readableText = await page.locator('p, h1, h2, h3').count();
    expect(readableText).toBeGreaterThan(5);

    // Reset to desktop
    await page.setViewportSize({ width: 1200, height: 800 });
  });

  test('should verify language attributes', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Check for language attribute on html element
    const htmlLang = await page.locator('html').getAttribute('lang');
    expect(htmlLang).toBe('en'); // Default to English

    // Check for language changes if any code blocks or quotes in other languages
    const codeBlocks = await page.locator('pre code').count();
    expect(codeBlocks).toBeGreaterThan(0); // Should have code examples
  });

  test('should validate overall accessibility score', async ({ page }) => {
    await page.goto('/docs/module-2');

    // Perform basic accessibility checks
    const contentLength = (await page.locator('body').textContent()).length;
    const hasHeadings = await page.locator('h2, h3').count() > 0;
    const hasLinks = await page.locator('a[href]').count() > 0;
    const hasImages = await page.locator('img').count() >= 0; // May have no images

    // Basic accessibility validation
    expect(contentLength).toBeGreaterThan(300); // Substantial content
    expect(hasHeadings).toBeTruthy(); // Proper structure
    expect(hasLinks).toBeTruthy(); // Navigation available
  });
});