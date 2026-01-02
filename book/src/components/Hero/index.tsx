import React, { useRef } from 'react';
import Link from '@docusaurus/Link';
import { useGSAP } from '@gsap/react';
import gsap from 'gsap';
import { ArrowRight, Github, Terminal, Cpu, Zap, BookOpen } from 'lucide-react';
import Translate from '@docusaurus/Translate';

export default function Hero() {
  const containerRef = useRef<HTMLElement>(null);
  const badgeRef = useRef<HTMLDivElement>(null);
  const titleRef = useRef<HTMLHeadingElement>(null);
  const subtitleRef = useRef<HTMLParagraphElement>(null);
  const buttonsRef = useRef<HTMLDivElement>(null);

  useGSAP(() => {
    const tl = gsap.timeline({ defaults: { ease: 'power3.out' } });

    tl.from(badgeRef.current, {
      y: 20,
      opacity: 0,
      duration: 0.8,
    })
    .from(titleRef.current, {
      y: 30,
      opacity: 0,
      duration: 0.8,
    }, '-=0.6')
    .from(subtitleRef.current, {
      y: 20,
      opacity: 0,
      duration: 0.8,
    }, '-=0.6')
    .from(buttonsRef.current, {
      y: 20,
      opacity: 0,
      duration: 0.8,
    }, '-=0.6');

    // Continuous floating animation for background blobs (using CSS mostly, but we can enhance)
    
  }, { scope: containerRef });

  return (
    <section ref={containerRef} className="relative overflow-hidden py-20 lg:py-32 flex items-center justify-center">
      
      <div className="container relative z-10 mx-auto px-4">
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 items-center">
          <div className="text-center lg:text-left">
            {/* Badge */}
            <div ref={badgeRef} className="inline-flex items-center gap-2 px-3 py-1 mb-8 rounded-full bg-cyan-50/50 dark:bg-cyan-900/20 border border-cyan-200 dark:border-cyan-800 backdrop-blur-sm">
              <span className="flex h-2 w-2 relative">
                <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-cyan-400 opacity-75"></span>
                <span className="relative inline-flex rounded-full h-2 w-2 bg-cyan-500"></span>
              </span>
              <span className="text-xs font-semibold uppercase tracking-wide text-cyan-700 dark:text-cyan-300">
                <Translate id="hero.badge" description="Badge text on Hero section">Open Source Guide</Translate>
              </span>
            </div>

            {/* Headline */}
            <h1 ref={titleRef} className="hero-title-large font-extrabold tracking-tight mb-6 text-slate-900 dark:text-white">
              <Translate
                id="hero.title"
                description="Main title on Hero section"
                values={{
                  physicalAI: (
                    <span className="text-transparent bg-clip-text bg-gradient-to-r from-cyan-500 via-blue-600 to-purple-600 dark:from-cyan-400 dark:via-blue-500 dark:to-purple-500">
                      <Translate id="hero.title.physicalAI" description="Physical AI text">Physical AI</Translate>
                    </span>
                  ),
                  br: <br />,
                }}
              >
                {'Build the {physicalAI}{br}Future.'}
              </Translate>
            </h1>

            {/* Subtitle */}
            <p ref={subtitleRef} className="text-lg md:text-xl text-slate-600 dark:text-slate-400 max-w-2xl mb-10 leading-relaxed mx-auto lg:mx-0">
              <Translate
                id="hero.subtitle"
                description="Subtitle on Hero section"
                values={{
                  ros2: <span className="font-semibold text-slate-900 dark:text-slate-200">ROS 2</span>,
                  vla: <span className="font-semibold text-slate-900 dark:text-slate-200">Vision-Language-Action</span>,
                }}
              >
                {'From {ros2} fundamentals to {vla} models. A comprehensive engineering handbook for the next generation of humanoid robotics.'}
              </Translate>
            </p>

            {/* Buttons */}
            <div ref={buttonsRef} className="flex flex-col sm:flex-row items-center justify-center lg:justify-start gap-4 mb-16">
              <Link
                to="/docs/intro"
                className="group relative inline-flex items-center justify-center px-8 py-3.5 text-base font-bold !text-white dark:!text-black button-text-override transition-all duration-200 bg-slate-900 dark:bg-white rounded-full hover:ring-4 hover:ring-slate-900/20 dark:hover:ring-white/20 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-slate-900"
              >
                <BookOpen className="w-5 h-5 mr-2" />
                <Translate id="hero.button.startReading" description="Start Reading button text">Start Reading</Translate>
                <ArrowRight className="w-4 h-4 ml-2 transition-transform group-hover:translate-x-1" />
              </Link>
              
              <Link
                href="https://github.com/hammadurrehman2006/Physical-AI-Humanoid-Robotics-Book"
                className="inline-flex items-center justify-center px-8 py-3.5 text-base font-bold !text-black dark:!text-white button-text-override transition-all duration-200 bg-white dark:bg-slate-800 border border-slate-200 dark:border-slate-700 rounded-full hover:bg-slate-50 dark:hover:bg-slate-700 hover:text-slate-900 dark:hover:text-white"
              >
                <Github className="w-5 h-5 mr-2" />
                <Translate id="hero.button.viewOnGithub" description="View on GitHub button text">View on GitHub</Translate>
              </Link>
            </div>
          </div>

          <img src="/img/group1.png" alt="Group 1" className="relative max-w-lg mx-auto w-full animate-float" />
        </div>
      </div>
    </section>
  );
}