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
    <section className="py-12 relative overflow-hidden">
      <div className="container mx-auto px-4">
        <div className="max-w-4xl mx-auto text-center mb-10">
          <h2 className="text-3xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white leading-tight">
            Complete <span className="text-transparent bg-clip-text bg-gradient-to-r from-cyan-500 to-purple-600">Learning Journey</span>
          </h2>
          <p className="text-lg text-slate-600 dark:text-slate-400 leading-relaxed max-w-3xl mx-auto">
            From Foundational Concepts to Cutting-Edge Embodied AI and Humanoid Robotics.
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 max-w-4xl mx-auto">
          {stats.map((stat, idx) => (
            <div 
              key={idx}
              className={clsx(
                "relative group p-5 rounded-2xl bg-slate-50 dark:bg-[#0F0F13] border transition-all duration-300 hover:-translate-y-1 hover:shadow-lg flex items-center text-left gap-4",
                stat.border
              )}
            >
              <div className={clsx(
                "w-12 h-12 rounded-lg flex items-center justify-center transition-colors shrink-0",
                stat.bg,
                stat.color
              )}>
                <stat.icon className="w-6 h-6" />
              </div>
              
              <div className="flex flex-col">
                <div className="text-2xl font-extrabold text-slate-900 dark:text-white leading-none mb-1">
                  {stat.value}
                </div>
                <div className="text-sm font-semibold uppercase tracking-wider text-slate-500 dark:text-slate-400 leading-none">
                  {stat.label}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}