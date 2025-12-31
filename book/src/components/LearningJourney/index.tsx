import React from 'react';
import clsx from 'clsx';
import { Calendar, Layers, BookOpen } from 'lucide-react';

const stats = [
  {
    value: '14-16',
    label: 'Weeks',
    icon: Calendar,
    color: 'text-cyan-500',
    bg: 'bg-cyan-500/10',
    border: 'border-cyan-200 dark:border-cyan-800'
  },
  {
    value: '4',
    label: 'Phases',
    icon: Layers,
    color: 'text-purple-500',
    bg: 'bg-purple-500/10',
    border: 'border-purple-200 dark:border-purple-800'
  },
  {
    value: '20+',
    label: 'Core Topics',
    icon: BookOpen,
    color: 'text-blue-500',
    bg: 'bg-blue-500/10',
    border: 'border-blue-200 dark:border-blue-800'
  },
];

export default function LearningJourney() {
  return (
    <section className="py-20 relative overflow-hidden">
      <div className="container mx-auto px-4">
        <div className="relative max-w-4xl mx-auto rounded-3xl overflow-hidden bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm border border-slate-200 dark:border-white/10 p-8 md:p-12">
          {/* Background Gradients */}
          <div className="absolute top-0 left-1/2 -translate-x-1/2 w-full h-full max-w-7xl pointer-events-none">
              <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-cyan-500/5 rounded-full blur-[100px]" />
              <div className="absolute bottom-1/4 right-1/4 w-96 h-96 bg-purple-500/5 rounded-full blur-[100px]" />
          </div>

          <div className="relative z-10">
            <div className="max-w-4xl mx-auto text-center mb-16">
              <h2 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white leading-tight">
                Complete <span className="text-transparent bg-clip-text bg-gradient-to-r from-cyan-500 to-purple-600">Learning Journey</span>
              </h2>
              <p className="text-xl font-medium text-slate-700 dark:text-slate-300 mb-4">
                From Foundational Concepts to Cutting-Edge Embodied AI and Humanoid Robotics.
              </p>
              <p className="text-lg text-slate-600 dark:text-slate-400 leading-relaxed max-w-3xl mx-auto">
                Embark on a transformative educational experience designed to take you from the basic principles of robotics to the advanced development of Vision-Language-Action (VLA) models for humanoid robots. This structured roadmap ensures a comprehensive understanding, progressively building your expertise to tackle the challenges of next-generation physical AI systems.
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-4xl mx-auto">
              {stats.map((stat, idx) => (
                <div 
                  key={idx}
                  className={clsx(
                    "relative group p-6 rounded-2xl bg-slate-50 dark:bg-[#0F0F13] border transition-all duration-300 hover:-translate-y-1 hover:shadow-xl",
                    stat.border
                  )}
                >
                  <div className={clsx(
                    "w-12 h-12 rounded-xl flex items-center justify-center mb-4 transition-colors",
                    stat.bg,
                    stat.color
                  )}>
                    <stat.icon className="w-6 h-6" />
                  </div>
                  
                  <div className="text-4xl font-extrabold mb-2 text-slate-900 dark:text-white">
                    {stat.value}
                  </div>
                  <div className="text-sm font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400">
                    {stat.label}
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}