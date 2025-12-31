const { chromium } = require('playwright');

(async () => {
  // Launch the browser
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage();

  try {
    // Navigate to the Urdu version of the homepage
    console.log('Navigating to Urdu homepage...');
    await page.goto('http://localhost:3000/ur/', { waitUntil: 'networkidle' });

    // Check that the html tag has lang='ur' and dir='ltr'
    const htmlLang = await page.evaluate(() => document.documentElement.lang);
    const htmlDir = await page.evaluate(() => document.documentElement.dir);
    const computedDir = await page.evaluate(() => window.getComputedStyle(document.documentElement).direction);

    console.log(`HTML lang attribute: ${htmlLang}`);
    console.log(`HTML dir attribute: ${htmlDir}`);
    console.log(`Computed direction: ${computedDir}`);

    // Verify the attributes
    if (htmlLang === 'ur' || htmlLang === 'ur-PK') {
      console.log('✅ HTML lang attribute is correctly set to "ur"');
    } else {
      console.log(`❌ HTML lang attribute is "${htmlLang}", expected "ur" or "ur-PK"`);
    }

    if (htmlDir === 'ltr' || computedDir === 'ltr') {
      console.log('✅ Direction is correctly set to LTR for Urdu content');
    } else {
      console.log(`❌ Direction is "${htmlDir}/${computedDir}", expected "ltr"`);
    }

    // Check body direction as well
    const bodyDir = await page.evaluate(() => document.body.dir);
    const bodyComputedDir = await page.evaluate(() => window.getComputedStyle(document.body).direction);

    console.log(`Body dir attribute: ${bodyDir}`);
    console.log(`Body computed direction: ${bodyComputedDir}`);

    if (bodyComputedDir === 'ltr') {
      console.log('✅ Body direction is correctly set to LTR for Urdu content');
    } else {
      console.log(`❌ Body direction is "${bodyComputedDir}", expected "ltr"`);
    }

    // Test with a specific content page as well
    console.log('\nNavigating to Urdu docs page...');
    await page.goto('http://localhost:3000/ur/docs/intro', { waitUntil: 'networkidle' });

    const introPageLang = await page.evaluate(() => document.documentElement.lang);
    const introPageDir = await page.evaluate(() => document.documentElement.dir);
    const introPageComputedDir = await page.evaluate(() => window.getComputedStyle(document.documentElement).direction);

    console.log(`Intro page lang attribute: ${introPageLang}`);
    console.log(`Intro page dir attribute: ${introPageDir}`);
    console.log(`Intro page computed direction: ${introPageComputedDir}`);

    if ((introPageLang === 'ur' || introPageLang === 'ur-PK') && (introPageDir === 'ltr' || introPageComputedDir === 'ltr')) {
      console.log('✅ Intro page correctly has Urdu lang and LTR direction');
    } else {
      console.log(`❌ Intro page - lang: "${introPageLang}", dir: "${introPageDir}", computed: "${introPageComputedDir}"`);
    }

    // Additional check: Verify that the text direction is indeed left-to-right
    // by checking a few text elements
    const textElements = await page.$$('.container div, .markdown p, .navbar__brand, .footer__title, .menu__list-item');
    let rtlDetected = false;

    for (const element of textElements.slice(0, 10)) { // Check first 10 elements only
      const computedStyle = await element.evaluate(el => window.getComputedStyle(el).direction);
      const textContent = await element.evaluate(el => el.textContent || '');

      if (textContent.trim()) { // Only check elements with text content
        console.log(`Element direction: ${computedStyle}, text sample: "${textContent.substring(0, 50)}..."`);
        if (computedStyle === 'rtl') {
          rtlDetected = true;
          console.log(`⚠️  RTL direction detected in element with text: "${textContent.substring(0, 100)}..."`);
        }
      }
    }

    if (!rtlDetected) {
      console.log('✅ No RTL directions detected in text elements - LTR layout maintained');
    } else {
      console.log('❌ Some elements still have RTL direction - LTR override not fully effective');
    }

    console.log('\nPlaywright test completed!');
    console.log('\nTest Summary:');
    console.log(`- HTML lang attribute: ${htmlLang}`);
    console.log(`- HTML direction (attr): ${htmlDir}`);
    console.log(`- HTML direction (computed): ${computedDir}`);
    console.log(`- Body direction (computed): ${bodyComputedDir}`);
    console.log(`- Intro page lang: ${introPageLang}`);
    console.log(`- Intro page direction: ${introPageComputedDir}`);
    console.log(`- RTL elements detected: ${rtlDetected ? 'Yes' : 'No'}`);

  } catch (error) {
    console.error('Playwright test failed with error:', error);
  } finally {
    // Close the browser after a delay to allow viewing
    await new Promise(resolve => setTimeout(resolve, 2000));
    await browser.close();
  }
})();