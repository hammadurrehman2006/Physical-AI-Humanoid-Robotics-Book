import React, {type ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useThemeConfig} from '@docusaurus/theme-common';
import { Bot } from "lucide-react";
import type {Props} from '@theme/Logo';

export default function Logo(props: Props): ReactNode {
  const {
    navbar: {title: navbarTitle, logo},
  } = useThemeConfig();

  const {imageClassName, titleClassName, ...propsRest} = props;
  const logoLink = useBaseUrl(logo?.href || '/');

  return (
    <Link
      to={logoLink}
      {...propsRest}
      {...(logo?.target && {target: logo.target})}
      className={`${propsRest.className || ''} flex items-center no-underline hover:no-underline`}
    >
      {/* Custom Robot Head Icon */}
      <div
        style={{
          width: '50px',
          height: '50px',
          background: 'linear-gradient(135deg, #4ade80, #15803d)', // Green gradient
          borderRadius: '12px',
          boxShadow: '0 4px 15px rgba(74, 222, 128, 0.5)', // Shining glow
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          marginRight: '12px'
        }}
        className="flex-shrink-0"
      >
        <Bot size={32} color="white" />
      </div>

      {/* Standard Title Rendering */}
      {navbarTitle != null && <b className={titleClassName}>{navbarTitle}</b>}
    </Link>
  );
}