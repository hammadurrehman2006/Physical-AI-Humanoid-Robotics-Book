import React, { useRef } from 'react';
import Link from '@docusaurus/Link';
import { useGSAP } from '@gsap/react';
import gsap from 'gsap';
import { ArrowRight, Github, Terminal, Cpu, Zap, BookOpen } from 'lucide-react';

export default function Hero() {
  const containerRef = useRef<HTMLElement>(null);
  const badgeRef = useRef<HTMLDivElement>(null);
  const titleRef = useRef<HTMLHeadingElement>(null);
  const subtitleRef = useRef<HTMLParagraphElement>(null);
  const buttonsRef = useRef<HTMLDivElement>(null);
  const terminalRef = useRef<HTMLDivElement>(null);
  const floatingIconsRef = useRef<HTMLDivElement>(null);

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
    }, '-=0.6')
    .from(terminalRef.current, {
      y: 40,
      opacity: 0,
      duration: 1,
      ease: 'back.out(1.2)',
    }, '-=0.4');

    // Continuous floating animation for background blobs (using CSS mostly, but we can enhance)
    
  }, { scope: containerRef });

  return (
    <section ref={containerRef} className="relative overflow-hidden bg-white dark:bg-[#0F0F13] min-h-screen flex items-center justify-center">
      
      {/* Background Gradients */}
      <div className="absolute top-0 left-0 w-full h-full overflow-hidden pointer-events-none">
        <div className="absolute top-[-10%] right-[-5%] w-[500px] h-[500px] bg-cyan-500/20 rounded-full blur-[100px] opacity-50 mix-blend-multiply dark:mix-blend-screen animate-blob" />
        <div className="absolute top-[-10%] left-[-10%] w-[500px] h-[500px] bg-purple-500/20 rounded-full blur-[100px] opacity-50 mix-blend-multiply dark:mix-blend-screen animate-blob animation-delay-2000" />
        <div className="absolute bottom-[-20%] left-[20%] w-[600px] h-[600px] bg-blue-500/20 rounded-full blur-[120px] opacity-50 mix-blend-multiply dark:mix-blend-screen animate-blob animation-delay-4000" />
      </div>

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
                Open Source Guide
              </span>
            </div>

            {/* Headline */}
            <h1 ref={titleRef} className="hero-title-large font-extrabold tracking-tight mb-6 text-slate-900 dark:text-white">
              Build the <span className="text-transparent bg-clip-text bg-gradient-to-r from-cyan-500 via-blue-600 to-purple-600 dark:from-cyan-400 dark:via-blue-500 dark:to-purple-500">Physical AI</span><br />
              Future.
            </h1>

            {/* Subtitle */}
            <p ref={subtitleRef} className="text-lg md:text-xl text-slate-600 dark:text-slate-400 max-w-2xl mb-10 leading-relaxed mx-auto lg:mx-0">
              From <span className="font-semibold text-slate-900 dark:text-slate-200">ROS 2</span> fundamentals to <span className="font-semibold text-slate-900 dark:text-slate-200">Vision-Language-Action</span> models. 
              A comprehensive engineering handbook for the next generation of humanoid robotics.
            </p>

            {/* Buttons */}
            <div ref={buttonsRef} className="flex flex-col sm:flex-row items-center justify-center lg:justify-start gap-4 mb-16">
              <Link
                to="/docs/intro"
                className="group relative inline-flex items-center justify-center px-8 py-3.5 text-base font-bold text-white transition-all duration-200 bg-slate-900 dark:bg-white dark:text-slate-900 rounded-full hover:ring-4 hover:ring-slate-900/20 dark:hover:ring-white/20 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-slate-900"
              >
                <BookOpen className="w-5 h-5 mr-2" />
                Start Reading
                <ArrowRight className="w-4 h-4 ml-2 transition-transform group-hover:translate-x-1" />
              </Link>
              
              <Link
                href="https://github.com/your-username/physical-ai-book"
                className="inline-flex items-center justify-center px-8 py-3.5 text-base font-bold text-slate-700 dark:text-slate-200 transition-all duration-200 bg-white dark:bg-slate-800 border border-slate-200 dark:border-slate-700 rounded-full hover:bg-slate-50 dark:hover:bg-slate-700 hover:text-slate-900 dark:hover:text-white"
              >
                <Github className="w-5 h-5 mr-2" />
                View on GitHub
              </Link>
            </div>
          </div>

          {/* Decorative Terminal/Code Element */}
          <div ref={terminalRef} className="relative max-w-4xl mx-auto w-full">
            {/* Glassmorphism Container */}
            <div className="relative rounded-xl overflow-hidden bg-slate-900/90 shadow-2xl ring-1 ring-white/10">
              
              {/* Window Controls */}
              <div className="flex items-center gap-2 px-4 py-3 bg-white/5 border-b border-white/5">
                <div className="w-3 h-3 rounded-full bg-red-500/80" />
                <div className="w-3 h-3 rounded-full bg-yellow-500/80" />
                <div className="w-3 h-3 rounded-full bg-green-500/80" />
                <div className="flex-1 text-center">
                  <span className="text-xs font-mono text-slate-400">humanoid_brain.py</span>
                </div>
              </div>

              {/* Code Content */}
              <div className="p-6 text-left overflow-hidden">
                <pre className="font-mono text-sm md:text-base leading-relaxed">
                  <code className="block">
                    <span className="text-purple-400">import</span> <span className="text-blue-400">rclpy</span>{'\n'}
                    <span className="text-purple-400">from</span> <span className="text-blue-400">ai_models</span> <span className="text-purple-400">import</span> Brain{'\n'}
                    {'\n'}
                    <span className="text-yellow-400">class</span> <span className="text-blue-300">Robot</span>(Node):{'\n'}
                    {'    '}<span className="text-purple-400">async def</span> <span className="text-blue-300">think</span>(self, obs):{'\n'}
                    {'        '}action = <span className="text-purple-400">await</span> self.brain.infer(obs){'\n'}
                    {'        '}self.move(action){'\n'}
                  </code>
                </pre>
              </div>
            </div>
            
            {/* Floating Icons behind */}
            <div className="absolute -top-12 -left-12 p-4 bg-slate-800 rounded-2xl shadow-xl transform -rotate-12 animate-float hidden md:block border border-slate-700">
              <Terminal className="w-8 h-8 text-cyan-400" />
            </div>
            <div className="absolute -bottom-8 -right-8 p-4 bg-slate-800 rounded-2xl shadow-xl transform rotate-6 animate-float animation-delay-2000 hidden md:block border border-slate-700">
              <Cpu className="w-8 h-8 text-purple-400" />
            </div>

          </div>
        </div>
      </div>
    </section>
  );
}