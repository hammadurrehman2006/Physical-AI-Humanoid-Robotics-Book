/**
 * Font loading utilities for Urdu and other language fonts
 * Provides optimized font loading strategies for better performance
 */

/**
 * Preload font faces for a given font family
 * @param {string} fontFamily - The font family to preload
 * @returns {Promise<void>}
 */
export const preloadFont = async (fontFamily) => {
  if (typeof document !== 'undefined' && document.fonts) {
    try {
      // Create a temporary text to test font loading
      const text = 'abcdefghijklmnopqrstuvwxyz';
      const fontFace = new FontFaceObserver(fontFamily, { weight: 400 });
      await fontFace.load(text, { timeout: 5000 });
      return true;
    } catch (error) {
      console.warn(`Font ${fontFamily} could not be loaded:`, error);
      return false;
    }
  }
  return false;
};

/**
 * Optimize font loading by preloading critical fonts
 */
export const optimizeFontLoading = () => {
  // Preload critical fonts for better performance
  if (typeof document !== 'undefined') {
    // Add font display swap to improve loading performance
    const style = document.createElement('style');
    style.textContent = `
      @font-face {
        font-family: 'Jameel Noori Nastaleeq';
        font-display: swap;
      }
      @font-face {
        font-family: 'Gulzar';
        font-display: swap;
      }
      @font-face {
        font-family: 'Noto Nastaliq Urdu';
        font-display: swap;
      }
      @font-face {
        font-family: 'Noto Sans Arabic';
        font-display: swap;
      }
      @font-face {
        font-family: 'Noto Sans Urdu';
        font-display: swap;
      }
    `;
    document.head.appendChild(style);
  }
};

/**
 * Initialize font loading optimization
 */
export const initializeFontOptimization = () => {
  // Optimize font loading when the document is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', optimizeFontLoading);
  } else {
    optimizeFontLoading();
  }
};

// Initialize font optimization if in browser environment
if (typeof window !== 'undefined') {
  initializeFontOptimization();
}