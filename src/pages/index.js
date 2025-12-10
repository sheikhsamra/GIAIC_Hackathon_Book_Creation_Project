import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <div className="text--center margin-bottom--lg">
              <div className="margin-bottom--lg">
                <img
                  src="/img/robotics-logo.svg"
                  alt="Physical AI & Humanoid Robotics Logo"
                  className={styles.heroLogo}
                  style={{maxWidth: '150px', height: 'auto', margin: '0 auto'}}
                />
              </div>
            </div>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/module-1">
                Start Learning Now
              </Link>
              <Link
                className="button button--primary button--lg margin-left--sm"
                to="/docs/module-1/learning-objectives">
                View Learning Objectives
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function AdditionalInfo() {
  return (
    <section className="margin-top--lg">
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>🤖 Comprehensive Modules</h3>
              <p>6 detailed modules from ROS 2 fundamentals to Vision-Language-Action systems, covering all aspects of Physical AI and Humanoid Robotics.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>🛠️ Hands-On Labs</h3>
              <p>Practical exercises with safety considerations, real-world applications, and simulation environments using Gazebo and NVIDIA Isaac.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>🚀 Capstone Project</h3>
              <p>Integrate all concepts in a comprehensive final project building intelligent humanoid robotics systems.</p>
            </div>
          </div>
        </div>

        <div className="row margin-top--lg">
          <div className="col col--6">
            <div className="padding-horiz--md">
              <h3>📚 Learning Features</h3>
              <ul>
                <li>Detailed theory with practical applications</li>
                <li>Code examples in Python, C++, and ROS 2</li>
                <li>Interactive labs and quizzes</li>
                <li>Diagrams and visualizations</li>
                <li>Urdu translations for accessibility</li>
              </ul>
            </div>
          </div>
          <div className="col col--6">
            <div className="padding-horiz--md">
              <h3>🎯 Target Audience</h3>
              <ul>
                <li>Computer Science and Engineering students</li>
                <li>Robotics researchers and professionals</li>
                <li>AI/ML engineers interested in robotics</li>
                <li>Anyone interested in Physical AI concepts</li>
                <li>Developers working with ROS 2 and simulation</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className="margin-top--lg padding-top--lg padding-bottom--lg" style={{backgroundColor: '#f8f9fa'}}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2 text--center">
            <h2>Ready to Start Your Journey in Physical AI & Humanoid Robotics?</h2>
            <p className="margin-top--md">Begin with Module 1 to learn the fundamentals of Physical AI and build up to advanced humanoid robotics systems.</p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/module-1">
                Start Learning
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics Textbook`}
      description="Comprehensive Guide to Physical AI, ROS 2, Gazebo, and NVIDIA Isaac">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <AdditionalInfo />
        <CTASection />
      </main>
    </Layout>
  );
}
