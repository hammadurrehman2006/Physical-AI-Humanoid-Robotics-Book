import type { ReactNode } from "react";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";
import Marquee from "@site/src/components/Marquee";
import Hero from "../components/Hero";
import LearningJourney from "@site/src/components/LearningJourney";
import CallToAction from "@site/src/components/CallToAction";
import Translate, { translate } from "@docusaurus/Translate";

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={translate({
        id: "homepage.title",
        message: `Hello from ${siteConfig.title}`,
        description: "The homepage title",
      })}
      description={translate({
        id: "homepage.description",
        message: "Description will go into a meta tag in <head />",
        description: "The homepage description",
      })}
    >
      <div id="tw-scope">
        <div className="relative overflow-hidden bg-white dark:bg-[#0F0F13]">
          {/* Background Gradients - Global for Home */}
          <div className="absolute top-0 left-0 w-full h-full overflow-hidden pointer-events-none z-0">
            <div className="absolute top-[-10%] right-[-5%] w-[500px] h-[500px] bg-cyan-500/20 rounded-full blur-[100px] opacity-50 mix-blend-multiply dark:mix-blend-screen animate-blob" />
            <div className="absolute top-[-10%] left-[-10%] w-[500px] h-[500px] bg-purple-500/20 rounded-full blur-[100px] opacity-50 mix-blend-multiply dark:mix-blend-screen animate-blob animation-delay-2000" />
            <div className="absolute bottom-[-20%] left-[20%] w-[600px] h-[600px] bg-blue-500/20 rounded-full blur-[120px] opacity-50 mix-blend-multiply dark:mix-blend-screen animate-blob animation-delay-4000" />
          </div>

          <div className="relative z-10">
            <Hero />
            <Marquee
              text={translate({
                id: "homepage.marquee",
                message: "Physical AI & Humanoid Robotics",
                description: "Text scrolling in the marquee",
              })}
            />
            <main>
              <HomepageFeatures />
              <LearningJourney />
              <CallToAction />
            </main>
          </div>
        </div>
      </div>
    </Layout>
  );
}
