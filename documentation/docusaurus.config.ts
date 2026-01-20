import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

const config: Config = {
  title: 'Opoch',
  tagline: 'Theory of Everything - Proofs & Research',
  favicon: 'img/favicon.svg',

  // Disabled v4 future flag - was causing mobile menu issues
  // future: {
  //   v4: true,
  // },

  url: 'https://docs.opoch.com',
  baseUrl: '/',

  organizationName: 'opoch',
  projectName: 'documentation',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/', // Docs at root
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
        },
        blog: false, // Disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
      type: 'text/css',
      integrity: 'sha384-n8MVd4RsNIU0tAv4ct0nTaAbDJwPJzDEaqSD1odI+WdtXRGWt2kTvGFasHpSy3SV',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig: {
    image: 'img/opoch-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: true, // Force dark mode only
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: 'Opoch',
      logo: {
        alt: 'Opoch Logo',
        src: 'img/opoch-logo.svg',
        href: 'https://www.opoch.com',
        target: '_self',
      },
      items: [
        {
          to: '/truth/introduction',
          label: 'The Mathematics - Truth',
          position: 'left',
        },
        {
          to: '/proof',
          label: 'Evidence Pack',
          position: 'left',
        },
        {
          to: '/technology/overview',
          label: 'Technology',
          position: 'left',
        },
        {
          to: '/resources',
          label: 'More',
          position: 'left',
        },
        {
          to: '/verify',
          label: 'Verify with ChatGPT',
          position: 'right',
          className: 'navbar__link--highlight',
        },
        {
          href: 'https://www.opoch.com',
          label: 'Home',
          position: 'right',
        },
        {
          href: 'https://chat.opoch.com',
          label: 'Try It',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'The Mathematics',
          items: [
            {
              label: 'Introduction',
              to: '/truth/introduction',
            },
            {
              label: 'The Derivation',
              to: '/proof/derivations/core-logic/the-derivation',
            },
            {
              label: 'Null-State Logic',
              to: '/proof/derivations/core-logic/opoch-kernel',
            },
          ],
        },
        {
          title: 'Proofs',
          items: [
            {
              label: 'Derivations',
              to: '/proof/derivations/core-logic/the-derivation',
            },
            {
              label: 'Falsification',
              to: '/proof/falsification/math-logic/godel-incompleteness',
            },
            {
              label: 'Validation',
              to: '/proof/validation/introduction',
            },
          ],
        },
        {
          title: 'Opoch',
          items: [
            {
              label: 'Home',
              href: 'https://www.opoch.com',
            },
            {
              label: 'Try It',
              href: 'https://chat.opoch.com',
            },
            {
              label: 'Falsify Us',
              href: 'https://www.opoch.com/falsify',
            },
          ],
        },
        {
          title: 'Connect',
          items: [
            {
              label: 'X / Twitter',
              href: 'https://x.com/opaborsh',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Opoch. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'latex'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
