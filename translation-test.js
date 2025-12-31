const { test, expect } = require('@playwright/test');

test.describe('Urdu Translation Functionality', () => {
  test('should load translation test page', async ({ page }) => {
    await page.goto('http://localhost:3000/translation-test');

    // Check if the page loads correctly
    await expect(page.locator('h1')).toContainText('Translation Functionality Test');
    console.log('✓ Translation test page loaded successfully');
  });

  test('should display Urdu text with proper styling', async ({ page }) => {
    await page.goto('http://localhost:3000/translation-test');

    // Wait for the page to load
    await page.waitForSelector('.urdu-text-renderer');

    // Check if Urdu text is rendered
    const urduText = page.locator('.urdu-text-renderer').first();
    await expect(urduText).toBeVisible();

    // Check if the text direction is LTR (even for Urdu)
    const computedStyle = await urduText.evaluate(node => window.getComputedStyle(node).direction);
    expect(computedStyle).toBe('ltr');
    console.log('✓ Urdu text is displayed with LTR direction');
  });

  test('should have language switcher functionality', async ({ page }) => {
    await page.goto('http://localhost:3000/translation-test');

    // Check for language switcher buttons
    const englishBtn = page.locator('button:has-text("English")');
    const urduBtn = page.locator('button:has-text("اردو")');

    await expect(englishBtn).toBeVisible();
    await expect(urduBtn).toBeVisible();
    console.log('✓ Language switcher buttons are visible');
  });

  test('should handle locale switching', async ({ page }) => {
    // Test English locale
    await page.goto('http://localhost:3000');
    const englishH1 = page.locator('h1');
    await expect(englishH1).toBeVisible();

    // Test Urdu locale
    await page.goto('http://localhost:3000/ur/');
    const urduH1 = page.locator('h1');
    await expect(urduH1).toBeVisible();
    console.log('✓ Both English and Urdu locales are accessible');
  });
});