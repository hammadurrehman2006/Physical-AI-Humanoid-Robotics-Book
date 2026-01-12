// book/src/theme/ur/UrduLayout.tsx
import React from 'react';
import UrduContentWrapper from '../UrduContentWrapper';

interface UrduLayoutProps {
  children: React.ReactNode;
}

const UrduLayout: React.FC<UrduLayoutProps> = ({ children }) => {
  return <UrduContentWrapper>{children}</UrduContentWrapper>;
};

export default UrduLayout;