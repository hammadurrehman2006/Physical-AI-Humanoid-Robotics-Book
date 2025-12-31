import React from 'react';
import styles from './styles.module.css';

export default function Marquee({ text = "Physical AI & Humanoid Robotics" }: { text?: string }) {
  // Repeat the text multiple times to ensure it covers wide screens
  const content = (
    <>
      <span className={styles.item}>{text}</span>
      <span className={styles.separator}>•</span>
      <span className={styles.item}>{text}</span>
      <span className={styles.separator}>•</span>
      <span className={styles.item}>{text}</span>
      <span className={styles.separator}>•</span>
      <span className={styles.item}>{text}</span>
      <span className={styles.separator}>•</span>
    </>
  );

  return (
    <div className={styles.marqueeWrapper}>
      <div className={styles.marqueeTrack}>
        <div className={styles.marqueeContent}>
          {content}
        </div>
        {/* Duplicate content for seamless loop */}
        <div className={styles.marqueeContent}>
          {content}
        </div>
      </div>
    </div>
  );
}
