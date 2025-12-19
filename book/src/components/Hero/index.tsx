import React from 'react';
import { motion } from 'framer-motion';
import { Sparkles, Play, ArrowRight, BookOpen, Cpu, Network } from 'lucide-react';
import styles from './styles.module.css';

function Hero() {
  return (
    <section className={`relative z-10 max-w-7xl mx-auto px-6 py-16 md:py-24 ${styles.heroContainer}`}>
      <div className="grid md:grid-cols-2 gap-12 items-center">
        {/* Left Content */}
        <motion.div
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
        >
          <div className="inline-flex items-center gap-2 bg-cyan-500/10 border border-cyan-500/30 rounded-full px-4 py-2 mb-6">
            <Sparkles className="w-4 h-4 text-cyan-500 dark:text-cyan-400" />
            <span className="text-cyan-600 dark:text-cyan-400 text-sm">
              Interactive Learning Platform
            </span>
          </div>

          <h1 className="text-4xl md:text-5xl lg:text-6xl mb-6">
            <span className="bg-gradient-to-r from-gray-900 via-cyan-700 to-blue-700 dark:from-white dark:via-cyan-200 dark:to-blue-400 bg-clip-text text-transparent">
              Master Physical AI & Humanoid Robotics
            </span>
          </h1>

          <p className="text-gray-600 dark:text-gray-400 text-lg md:text-xl mb-8 leading-relaxed">
            Dive deep into embodied intelligence, from ROS 2 fundamentals to
            cutting-edge Vision-Language-Action models. Build the future of
            robotics.
          </p>

          <div className="flex flex-col sm:flex-row gap-4 mb-8">
            <button className="bg-gradient-to-r from-cyan-500 to-blue-600 hover:from-cyan-400 hover:to-blue-500 px-8 py-4 rounded-xl transition-all flex items-center justify-center gap-3 shadow-lg shadow-cyan-500/30 hover:shadow-cyan-500/50 group text-white">
              <Play className="w-5 h-5 group-hover:scale-110 transition-transform" />
              <span>Start Learning</span>
              <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
            </button>
            <button className="bg-gray-100 dark:bg-gray-800/50 backdrop-blur-sm border border-gray-300 dark:border-cyan-500/30 hover:border-gray-400 dark:hover:border-cyan-500/60 px-8 py-4 rounded-xl transition-all flex items-center justify-center gap-3 text-gray-900 dark:text-white">
              <BookOpen className="w-5 h-5" />
              <span>View Modules</span>
            </button>
          </div>

          {/* Stats */}
          <div className="grid grid-cols-3 gap-4">
            <div className="bg-white dark:bg-gray-900/40 backdrop-blur-sm border border-gray-200 dark:border-cyan-500/20 rounded-xl p-4 shadow-sm">
              <div className="text-2xl md:text-3xl bg-gradient-to-r from-cyan-500 to-cyan-600 bg-clip-text text-transparent mb-1">
                4
              </div>
              <div className="text-gray-500 dark:text-gray-500 text-sm">
                Modules
              </div>
            </div>
            <div className="bg-white dark:bg-gray-900/40 backdrop-blur-sm border border-gray-200 dark:border-cyan-500/20 rounded-xl p-4 shadow-sm">
              <div className="text-2xl md:text-3xl bg-gradient-to-r from-blue-500 to-blue-600 bg-clip-text text-transparent mb-1">
                50+
              </div>
              <div className="text-gray-500 dark:text-gray-500 text-sm">
                Lessons
              </div>
            </div>
            <div className="bg-white dark:bg-gray-900/40 backdrop-blur-sm border border-gray-200 dark:border-cyan-500/20 rounded-xl p-4 shadow-sm">
              <div className="text-2xl md:text-3xl bg-gradient-to-r from-purple-500 to-purple-600 bg-clip-text text-transparent mb-1">
                12h
              </div>
              <div className="text-gray-500 dark:text-gray-500 text-sm">
                Content
              </div>
            </div>
          </div>
        </motion.div>

        {/* Right - Robot Image */}
        <motion.div
          initial={{ opacity: 0, scale: 0.8 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.8, delay: 0.2 }}
          className="relative"
        >
          <div className="relative rounded-2xl overflow-hidden border border-gray-200 dark:border-cyan-500/30 shadow-2xl dark:shadow-cyan-500/20">
            <img
              src="https://images.unsplash.com/photo-1760629863094-5b1e8d1aae74?crop=entropy&cs=tinysrgb&fit=max&fm=jpg&ixid=M3w3Nzg4Nzd8MHwxfHNlYXJjaHwxfHxodW1hbm9pZCUyMHJvYm90JTIwZnV0dXJpc3RpY3xlbnwxfHx8fDE3NjYwNjU3MzF8MA&ixlib=rb-4.1.0&q=80&w=1080&utm_source=figma&utm_medium=referral"
              alt="Humanoid Robot"
              className="w-full h-auto"
            />
            {/* Overlay gradient */}
            <div className="absolute inset-0 bg-gradient-to-t from-white dark:from-gray-900 via-transparent to-transparent"></div>

            {/* Floating elements */}
            <motion.div
              animate={{ y: [0, -10, 0] }}
              transition={{ duration: 3, repeat: Infinity }}
              className="absolute top-8 right-8 bg-cyan-500/20 backdrop-blur-md border border-cyan-500/50 rounded-xl p-3 shadow-lg"
            >
              <Cpu className="w-6 h-6 text-cyan-500 dark:text-cyan-400" />
            </motion.div>
            <motion.div
              animate={{ y: [0, 10, 0] }}
              transition={{ duration: 3, repeat: Infinity, delay: 1 }}
              className="absolute bottom-8 left-8 bg-blue-500/20 backdrop-blur-md border border-blue-500/50 rounded-xl p-3 shadow-lg"
            >
              <Network className="w-6 h-6 text-blue-500 dark:text-blue-400" />
            </motion.div>
          </div>

          {/* Decorative elements */}
          <div className="absolute -top-4 -right-4 w-24 h-24 bg-cyan-500/10 dark:bg-cyan-500/20 rounded-full blur-2xl"></div>
          <div className="absolute -bottom-4 -left-4 w-32 h-32 bg-blue-500/10 dark:bg-blue-500/20 rounded-full blur-2xl"></div>
        </motion.div>
      </div>
    </section>
  );
}

export default Hero;