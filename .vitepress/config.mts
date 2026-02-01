import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "Foxy Robot",
  description: "Documentation website for the open-source robot foxy-robot",
  base: '/foxy-robot/',
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    // nav: [
    //   { text: 'Home', link: '/README' },
    //   { text: 'Documentation', link: '/README' }
    // ],

    sidebar: [
      {
        text: 'Start Up',
        items: [
          { text: 'Installation', link: '/docs/installation' },
          { text: 'Getting Started', link: '/docs/getting-started' },
        ]
      },
      {
        text: 'Exercises',
        items: [
          { text: 'Braitenberg', link: '/docs/exercises/braitenberg' },
        ]
      },
      {
        text: 'For Developers',
        items: [
          { text: 'Developer guide', link: '/docs/developers/guide' },
        ]
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/EOLab-HSRW/foxy-robot' }
    ]
  },
  rewrites: {
    'README.md': 'index.md',
  }
})
