import React from 'react';
import Link from '@docusaurus/Link';
import { ArrowRight, Sparkles } from 'lucide-react';
import { motion } from 'framer-motion';

export default function CallToAction() {
  return (
    <section className="py-24 relative overflow-hidden">
      <div className="container mx-auto px-4">
        <motion.div 
          initial={{ opacity: 0, y: 50 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          transition={{ duration: 0.8 }}
          className="relative max-w-6xl mx-auto rounded-[2rem] overflow-hidden bg-gradient-to-br from-cyan-50 via-blue-50 to-purple-50 dark:from-slate-900 dark:via-slate-900 dark:to-slate-800 border border-cyan-200 dark:border-cyan-800 shadow-2xl"
        >
            {/* Background Effects */}
            <div className="absolute inset-0">
                <div className="absolute top-0 right-0 w-[500px] h-[500px] bg-gradient-to-bl from-purple-500/20 to-transparent blur-3xl opacity-60" />
                <div className="absolute bottom-0 left-0 w-[500px] h-[500px] bg-gradient-to-tr from-cyan-500/20 to-transparent blur-3xl opacity-60" />
            </div>

            <div className="relative z-10 px-8 py-20 md:py-24 text-center">
                <motion.div 
                  initial={{ opacity: 0, scale: 0.9 }}
                  whileInView={{ opacity: 1, scale: 1 }}
                  transition={{ delay: 0.2, duration: 0.5 }}
                  className="inline-flex items-center gap-2 px-4 py-1.5 mb-8 rounded-full bg-white/40 dark:bg-white/5 border border-cyan-200 dark:border-white/10 backdrop-blur-sm"
                >
                    <Sparkles className="w-4 h-4 text-cyan-600 dark:text-yellow-400" />
                    <span className="text-sm font-semibold text-slate-800 dark:text-white/90">
                        Join the Revolution
                    </span>
                </motion.div>

                <motion.h2 
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.3, duration: 0.6 }}
                  className="text-5xl md:text-6xl lg:text-7xl font-extrabold text-slate-900 dark:text-white mb-8 tracking-tight"
                >
                    Ready to Start Your Journey?
                </motion.h2>
                
                <motion.p 
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.4, duration: 0.6 }}
                  className="text-2xl text-slate-600 dark:text-slate-300 max-w-3xl mx-auto mb-12 leading-relaxed"
                >
                    Join thousands of developers building the future of embodied AI and humanoid robotics.
                </motion.p>

                <motion.div 
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.5, duration: 0.6 }}
                  className="flex flex-col sm:flex-row items-center justify-center gap-4"
                >
                    <Link
                        to="/docs/intro"
                        className="group relative inline-flex items-center justify-center px-8 py-3.5 text-base font-bold !text-white dark:!text-black button-text-override transition-all duration-200 bg-slate-900 dark:bg-white rounded-full hover:ring-4 hover:ring-slate-900/20 dark:hover:ring-white/20 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-slate-900 shadow-lg"
                    >
                        Begin Learning Now
                        <ArrowRight className="w-5 h-5 ml-2 transition-transform group-hover:translate-x-1" />
                    </Link>
                </motion.div>
            </div>
        </motion.div>
      </div>
    </section>
  );
}
