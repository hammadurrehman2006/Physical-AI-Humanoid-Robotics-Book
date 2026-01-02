import { ReactNode } from "react";
import { Cpu, Boxes, Brain, Eye, ArrowRight } from "lucide-react";
import Link from "@docusaurus/Link";
import clsx from "clsx";
import Translate, { translate } from "@docusaurus/Translate";

type ModuleItem = {
  title: string;
  icon: ReactNode;
  description: string;
  border: string;
  iconBg: string; // Gradient for the icon background
  link: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: translate({
      id: "homepage.features.module1.title",
      message: "Module 1: The Robotic Nervous System (ROS 2)",
      description: "Title for Module 1"
    }),
    icon: <Cpu className="w-8 h-8 text-white" />,
    description: translate({
      id: "homepage.features.module1.description",
      message: "Master the fundamental communication framework that powers modern robots. Learn nodes, topics, and real-time control.",
      description: "Description for Module 1"
    }),
    border: "border-cyan-200 dark:border-cyan-800",
    iconBg: "bg-gradient-to-br from-cyan-500 to-blue-600",
    link: "/docs/module-1"
  },
  {
    title: translate({
      id: "homepage.features.module2.title",
      message: "Module 2: The Digital Twin (Gazebo & Unity)",
      description: "Title for Module 2"
    }),
    icon: <Boxes className="w-8 h-8 text-white" />,
    description: translate({
      id: "homepage.features.module2.description",
      message: "Simulate physics and environments before deployment. Build high-fidelity virtual replicas for safe training.",
      description: "Description for Module 2"
    }),
    border: "border-emerald-200 dark:border-emerald-800",
    iconBg: "bg-gradient-to-br from-emerald-500 to-green-600",
    link: "/docs/module-2"
  },
  {
    title: translate({
      id: "homepage.features.module3.title",
      message: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)",
      description: "Title for Module 3"
    }),
    icon: <Brain className="w-8 h-8 text-white" />,
    description: translate({
      id: "homepage.features.module3.description",
      message: "Leverage GPU-accelerated simulation and reinforcement learning to train intelligent agents.",
      description: "Description for Module 3"
    }),
    border: "border-purple-200 dark:border-purple-800",
    iconBg: "bg-gradient-to-br from-purple-600 to-indigo-600",
    link: "/docs/module-3"
  },
  {
    title: translate({
      id: "homepage.features.module4.title",
      message: "Module 4: Vision-Language-Action (VLA)",
      description: "Title for Module 4"
    }),
    icon: <Eye className="w-8 h-8 text-white" />,
    description: translate({
      id: "homepage.features.module4.description",
      message: "Integrate Multimodal LLMs for embodied intelligence. Enable robots to see, understand, and act.",
      description: "Description for Module 4"
    }),
    border: "border-orange-200 dark:border-orange-800",
    iconBg: "bg-gradient-to-br from-orange-500 to-red-600",
    link: "/docs/module-4"
  },
];

function ModuleCard({ title, icon, description, border, iconBg, link }: ModuleItem) {
  const [mainTitle, subTitle] = title.split(':');

  return (
    <div className={clsx(
      "relative group flex flex-col h-full p-8 rounded-2xl bg-slate-50 dark:bg-[#0F0F13] border transition-all duration-300 hover:-translate-y-1 hover:shadow-xl",
      border
    )}>
      
      {/* Icon */}
      <div className={clsx(
        "w-14 h-14 rounded-2xl flex items-center justify-center mb-6 shadow-lg transform transition-transform duration-300 group-hover:scale-110",
        iconBg
      )}>
        {icon}
      </div>
      
      {/* Title */}
      <h3 className="text-xl font-bold mb-3 text-slate-900 dark:text-white leading-tight">
        {mainTitle}:<br/>
        <span className="text-slate-600 dark:text-slate-400 font-normal text-lg block mt-1">{subTitle}</span>
      </h3>
      
      {/* Description */}
      <p className="text-slate-600 dark:text-slate-400 mb-6 flex-grow leading-relaxed">
        {description}
      </p>
      
      {/* Link */}
      <Link
        to={link}
        className="inline-flex items-center gap-2 font-bold text-slate-900 dark:text-white group-hover:gap-3 transition-all duration-300"
      >
        <span>
          <Translate id="homepage.features.exploreModule">Explore Module</Translate>
        </span>
        <ArrowRight className="w-4 h-4" />
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className="py-20 relative overflow-hidden">
       <div className="container mx-auto px-4">
        <div className="max-w-4xl mx-auto text-center mb-16">
          <h2 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white leading-tight">
            <Translate id="homepage.features.course">Course</Translate> <span className="text-transparent bg-clip-text bg-gradient-to-r from-blue-600 to-cyan-500">
              <Translate id="homepage.features.curriculum">Curriculum</Translate>
            </span>
          </h2>
          <p className="text-xl font-medium text-slate-700 dark:text-slate-300 mb-4 max-w-2xl mx-auto">
            <Translate id="homepage.features.subtitle">
              A structured journey from foundational control systems to cutting-edge embodied AI.
            </Translate>
          </p>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 lg:gap-8 max-w-[90rem] mx-auto">
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
