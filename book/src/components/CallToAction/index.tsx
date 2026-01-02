import React from 'react';
import Link from '@docusaurus/Link';
import { ArrowRight, Sparkles } from 'lucide-react';
import Translate from '@docusaurus/Translate';

export default function CallToAction() {
  return (
    <section className="py-16 relative overflow-hidden">
      <div className="container mx-auto px-4 text-center">
        
        <div className="inline-flex items-center gap-2 px-3 py-1 mb-6 rounded-full bg-slate-100 dark:bg-slate-800 border border-slate-200 dark:border-slate-700">
            <Sparkles className="w-4 h-4 text-cyan-600 dark:text-cyan-400" />
            <span className="text-xs font-semibold text-slate-700 dark:text-slate-300">
                <Translate id="cta.badge" description="Badge text in Call to Action">
                  Join the Revolution
                </Translate>
            </span>
        </div>

        <h2 className="text-3xl md:text-5xl font-bold text-slate-900 dark:text-white mb-4 tracking-tight">
            <Translate id="cta.title" description="Main title in Call to Action">
              Ready to Start Your Journey?
            </Translate>
        </h2>
        
        <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto mb-8">
            <Translate id="cta.description" description="Description text in Call to Action">
              Join thousands of developers building the future of embodied AI and humanoid robotics.
            </Translate>
        </p>

        <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
            <Link
                to="/docs/intro"
                className="group relative inline-flex items-center justify-center px-6 py-3 text-sm font-bold !text-white dark:!text-black button-text-override transition-all duration-200 bg-slate-900 dark:bg-white rounded-full hover:ring-4 hover:ring-slate-900/20 dark:hover:ring-white/20 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-slate-900 shadow-md hover:shadow-lg"
            >
                <Translate id="cta.button" description="Button text in Call to Action">
                  Begin Learning Now
                </Translate>
                <ArrowRight className="w-4 h-4 ml-2 transition-transform group-hover:translate-x-1" />
            </Link>
        </div>
      </div>
    </section>
  );
}
