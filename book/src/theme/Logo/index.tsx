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
          width: '40px',
          height: '40px',
          background: 'linear-gradient(135deg, #4ade80, #15803d)', // Green gradient
          borderRadius: '10px',
          boxShadow: '0 3px 10px rgba(74, 222, 128, 0.4)', // Slightly smaller shadow
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          marginRight: '8px' // Reduced margin
        }}
        className="flex-shrink-0"
      >
        <Bot size={24} color="white" />
      </div>

      {/* Standard Title Rendering */}
      {navbarTitle != null && <b className={titleClassName}>{navbarTitle}</b>}
    </Link>
  );
}