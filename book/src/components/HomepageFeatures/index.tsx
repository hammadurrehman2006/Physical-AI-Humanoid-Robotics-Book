import { ReactNode } from "react";
import { Cpu, Boxes, Brain, Eye, ArrowRight } from "lucide-react";
import Link from "@docusaurus/Link";
import clsx from "clsx";
import styles from "./styles.module.css";

type ModuleItem = {
  title: string;
  icon: ReactNode;
  description: string;
  gradientClass: string;
  link: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: "Module 1: The Robotic Nervous System (ROS 2)",
    icon: <Cpu className={styles.iconSvg} />,
    description: "Master the fundamental communication framework that powers modern robots. Learn nodes, topics, and real-time control.",
    gradientClass: styles.gradientCyanBlue,
    link: "/docs/module-1"
  },
  {
    title: "Module 2: The Digital Twin (Gazebo & Unity)",
    icon: <Boxes className={styles.iconSvg} />,
    description: "Simulate physics and environments before deployment. Build high-fidelity virtual replicas for safe training.",
    gradientClass: styles.gradientEmeraldGreen,
    link: "/docs/module-2"
  },
  {
    title: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)",
    icon: <Brain className={styles.iconSvg} />,
    description: "Leverage GPU-accelerated simulation and reinforcement learning to train intelligent agents.",
    gradientClass: styles.gradientPurpleIndigo,
    link: "/docs/module-3"
  },
  {
    title: "Module 4: Vision-Language-Action (VLA)",
    icon: <Eye className={styles.iconSvg} />,
    description: "Integrate Multimodal LLMs for embodied intelligence. Enable robots to see, understand, and act.",
    gradientClass: styles.gradientOrangeRed,
    link: "/docs/module-4"
  },
];

function ModuleCard({ title, icon, description, gradientClass, link }: ModuleItem) {
  return (
    <div className={styles.cardWrapper}>
      {/* Glow Effect */}
      <div className={clsx(styles.cardGlow, gradientClass)}></div>
      
      {/* Card Content */}
      <div className={styles.cardContent}>
        <div className={clsx(styles.iconContainer, gradientClass)}>
          {icon}
        </div>
        
        <h3 className={styles.cardTitle}>
          {title.split(':')[0]}:<br/>
          <span className={styles.cardTitleSub}>{title.split(':')[1]}</span>
        </h3>
        
        <p className={styles.cardDescription}>
          {description}
        </p>
        
        <Link
          to={link}
          className={styles.cardLink}
        >
          <span>Explore Module</span>
          <ArrowRight className={styles.arrowIcon} />
        </Link>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.featuresSection}>
      <div className={styles.featuresContainer}>
        <div className={styles.header}>
          <h2 className={styles.headerTitle}>
            Course Curriculum
          </h2>
          <p className={styles.headerSubtitle}>
            A structured journey from foundational control systems to cutting-edge embodied AI.
          </p>
        </div>
        
        <div className={styles.grid}>
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
