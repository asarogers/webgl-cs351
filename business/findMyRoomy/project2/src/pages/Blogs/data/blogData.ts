// src/data/blogData.ts

export interface BlogPost {
  id: number;
  title: string;
  date: string;
  author: string;
  category: string;
  introText: string;
  excerpt: string;
  readTime: string;
  image: string;
  likes: number;
  comments: number;
  featured: boolean;
  tags: string[];
  sections: Array<{
    subtitle: string;
    content: string | Array<{ label: string; text: string }>;
    image?: string;
    video?: string;
  }>;
}

export const blogData: Record<number, BlogPost> = {
  1: {
    id: 1,
    title: "Roommate Compatibility: The Real Questions That Matter",
    date: "2025-06-10",
    author: "Sarah Chen",
    category: "Tips & Guides",
    introText:
      "Most roommate conflicts aren't about chores or noise — they're about mismatched values, habits, and expectations. This guide dives into the psychology of compatibility, with 15 real questions that predict co-living success. In this extended guide, we'll explore detailed scenarios, share personal stories from pairs of roommates who made it work (and those who didn't), and provide practical exercises to help you apply these insights in real time.",
    excerpt:
      "Stop wasting time with surface-level questions. These psychology-backed conversation starters reveal true compatibility and prevent future conflicts. Dive deeper into each query to unlock meaningful dialogue that lays the foundation for lasting harmony, and see sample transcripts to model your own discussions.",
    readTime: "8 min read",
    image:
      "https://images.unsplash.com/photo-1522202176988-66273c2fd55f?w=800&h=400&fit=crop",
    likes: 247,
    comments: 18,
    featured: true,
    tags: ["Compatibility", "Psychology", "Communication"],
    sections: [
      {
        subtitle: "Why Compatibility is Deeper Than Preferences",
        content: [
          {
            label: "The Surface Trap",
            text:
              "People often match based on music taste or favorite shows — but those rarely impact day-to-day living satisfaction. You might both love the same band yet clash over sleep schedules or cleaning habits.",
          },
          {
            label: "What Actually Matters",
            text:
              "Values around cleanliness, boundaries, emotional regulation, and communication styles are far more predictive of harmony. Disagreements over whether dishes can sit overnight reveal underlying differences in respect and responsibility.",
          },
          {
            label: "Daily Routine Sync",
            text:
              "Aligning daily schedules ensures that you respect each other's rhythms—whether it's an early morning workout or late-night work sessions. Misaligned routines are a frequent source of friction when one person's alarm goes off at 5 AM while the other is trying to sleep.",
          },
          {
            label: "Communication Frequency",
            text:
              "Some people prefer check-ins every few hours, others only once a day. Discussing how often you want to communicate about chores, groceries, or guests helps set clear expectations and avoid misunderstandings.",
          },
        ],
        image:
          "https://images.unsplash.com/photo-1522202176988-66273c2fd55f?w=800&h=400&fit=crop",
      },
      {
        subtitle: "15 Compatibility Questions That Reveal Red Flags",
        content:
          "We've compiled 15 questions based on therapist-approved roommate counseling practices. These include: 'How do you prefer to resolve conflict?', 'How do you define clean?', 'What does privacy mean to you?', and more. These aren't yes/no — they spark stories, which is where real insight lives.\n\nAdditional sample prompts:\n• How do you decompress after a long day?\n• What role does spontaneity play in your life?\n• How do you handle shared expenses on a tight budget?\n• Are you a saver or spender when it comes to household items?\n• What's your ideal balance between socializing and solitude?\n\nEach question is designed to surface core values and prevent surprises down the road.",
        video:
          "https://images.unsplash.com/photo-1560472354-b33ff0c44a43?w=800&h=400&fit=crop",
      },
      {
        subtitle: "The Psychology Behind Successful Co-Living",
        content:
          "Research shows that shared values predict roommate satisfaction better than shared interests. When people align on core principles like respect, communication, and responsibility, minor differences become manageable quirks rather than deal-breakers. In addition, longitudinal studies indicate that fairness in chore distribution and financial transparency are strong mediators of long-term happiness.\n\nCase Study: In one 12-month study, pairs who conducted weekly check-ins reported 65% fewer disputes compared to those who only chatted informally. That consistency builds trust — and trust is the glue of any shared household.",
      },
      {
        subtitle: "Conclusion: Choose Alignment, Not Just Affection",
        content:
          "Liking someone isn't enough. Co-living works when your habits and values align. Start hard conversations early — they pay off in months of peaceful coexistence and genuine friendship. Remember, compatibility is an ongoing practice, not a one-time checkbox. Schedule quarterly roommate retrospectives to celebrate wins and address simmering issues before they boil over.",
      },
      {
        subtitle: "Frequently Asked Questions",
        content: [
          {
            label: "Q: How do we start these tough conversations?",
            text:
              "Begin with a relaxed setting—over coffee or during a walk. Use open-ended prompts, listen actively, and avoid blame. Frame it as building a happier home together.",
          },
          {
            label: "Q: What if we disagree on a core value?",
            text:
              "A single difference isn't a deal-breaker—what matters is your willingness to compromise and establish a boundary that works for both.",
          },
          {
            label: "Q: How often should we revisit our agreements?",
            text:
              "Monthly check-ins are ideal in the first three months, then quarterly. Consistency prevents small annoyances from festering.",
          },
        ],
      },
      {
        subtitle: "Further Reading & Tools",
        content:
          "• Download our free compatibility worksheet: outlines questions and space for notes.\n• Recommended book: 'Connected Living' by Dr. Anne Wallace.\n• Podcast: 'Roommate Real Talk' — episodes on boundary setting and conflict resolution.\n• Online quiz: Take the 'Shared Space Values' assessment at myroomie.com/quiz.",
      },
    ],
  },

  2: {
    id: 2,
    title: "Building Trust in Roommate Apps: How We Verify Profiles",
    date: "2025-06-08",
    author: "Mike Rodriguez",
    category: "Safety & Trust",
    introText:
      "Fake roommate profiles lead to scams, ghosting, and bad living experiences. Here's how MyRoomie uses layered verification to create real trust — and how you can vet potential roommates safely. We’ll walk through each verification step in detail, show you dashboard screenshots, and share tips to spot fakes that slip through the cracks.",
    excerpt:
      "Learn the difference between minor quirks and major warning signs. Our data shows these 7 behaviors predict 89% of roommate conflicts. Discover the red-flag checklist top landlords use — and how our system automates it seamlessly.",
    readTime: "6 min read",
    image:
      "https://images.unsplash.com/photo-1560472354-b33ff0c44a43?w=800&h=400&fit=crop",
    likes: 189,
    comments: 25,
    featured: false,
    tags: ["Safety", "Verification", "Trust"],
    sections: [
      {
        subtitle: "Why Most Platforms Fail",
        content: [
          {
            label: "Weak Identity Checks",
            text:
              "Traditional apps rely on email or phone verification, which is easy to fake. We go further with government ID + selfie verification to confirm you are who you claim to be.",
          },
          {
            label: "Lack of Community Oversight",
            text:
              "Fake listings spread when no one is accountable. We counter this with active moderation, user flagging, and 24-hour response guarantees to investigate any suspicious activity.",
          },
          {
            label: "Shallow Background Checks",
            text:
              "Many apps don’t verify rental history or criminal records. We partner with trusted data providers to surface any major infractions, with the user’s consent, before you swipe right.",
          },
        ],
        video:
          "https://images.unsplash.com/photo-1551434678-e076c223a692?w=800&h=400&fit=crop",
      },
      {
        subtitle: "Our 3-Layer Verification Approach",
        content:
          "1. Government ID scan with liveness check to prevent spoofing.\n2. AI-based photo comparison to ensure the selfie matches the ID.\n3. Community flagging for ongoing safety — every flag triggers a manual review within 12 hours.\n\nSince launching, this model has reduced fake profiles by 94% and increased completed move-in rates by 37%.",
      },
      {
        subtitle: "Red Flags to Watch For",
        content:
          "Even with verification, stay alert for warning signs: reluctance to video chat, pressure to move quickly, requests for money upfront, or narratives that keep changing. Keep records of all communication, and trust your instincts if anything feels off.",
      },
      {
        subtitle: "Staying Safe When You Meet",
        content:
          "Even with verified matches, we recommend: always meet in a public place first (coffee shops or co-working spaces work great), video chat beforehand to see the person in action, and set clear expectations in writing (via email or chat). Safety is a shared responsibility — you get out what you put in.",
      },
      {
        subtitle: "Community Success Stories",
        content:
          "• Jane & Laura matched through our ID + photo process, now co-host monthly book clubs together.\n• Raj spotted a fake profile thanks to our flagging popup, preventing a $1,200 deposit loss.\n• Marcus discovered his perfect match while scrolling our safety dashboard during lunch break.",
      },
      {
        subtitle: "FAQs: Verification & Privacy",
        content: [
          {
            label: "Q: Is my ID data secure?",
            text:
              "Absolutely. We encrypt all uploads end-to-end and purge raw images after verification completes.",
          },
          {
            label: "Q: Can I opt out of community flagging?",
            text:
              "Flagging is voluntary but highly encouraged—more eyes make the platform safer for everyone.",
          },
          {
            label: "Q: What if I don’t have a government-issued ID?",
            text:
              "We support passports and national IDs from over 50 countries. If yours isn’t listed, contact support for manual review.",
          },
        ],
      },
      {
        subtitle: "Recommended Resources",
        content:
          "• Whitepaper on digital ID best practices: myroomie.com/whitepaper\n• Webinar: 'Trust & Safety in PropTech' — recorded session available on YouTube.\n• Blog series: 'Inside Verification' — upcoming deep dives on biometric liveness checks.",
      },
    ],
  },

  3: {
    id: 3,
    title: "Why 90% of Roommate Apps Fail (And How We're Different)",
    date: "2025-06-05",
    author: "Alex Kim",
    category: "Industry Insights",
    introText:
      "An inside look at the broken roommate industry, why existing solutions don't work, and how we're building something that actually helps people find compatible living situations. We’ll trace the rise and fall of five major apps, analyze user reviews, and show you exactly where each one missed the mark.",
    excerpt:
      "An insider’s guide to why legacy platforms disappoint renters and how a new generation of tools is rewriting the rules. We share data on user churn, average days-to-match, and satisfaction ratings to prove our point.",
    readTime: "12 min read",
    image:
      "https://images.unsplash.com/photo-1551434678-e076c223a692?w=800&h=400&fit=crop",
    likes: 342,
    comments: 47,
    featured: true,
    tags: ["Industry", "Product", "Innovation"],
    sections: [
      {
        subtitle: "The Fundamental Problems",
        content: [
          {
            label: "Surface-Level Matching",
            text:
              "Most apps focus on preferences (music, movies) rather than values (cleanliness, boundaries, communication). This leads to mismatched expectations and high churn rates.",
          },
          {
            label: "Lack of Safety Measures",
            text:
              "Weak verification systems allow fake profiles to flourish, creating unsafe environments and wasting users' time on no-shows or scams.",
          },
          {
            label: "Poor User Experience",
            text:
              "Clunky interfaces, limited search filters, and no meaningful conversation starters make it hard to connect authentically. Many users abandon apps after two weeks due to frustration.",
          },
          {
            label: "Data Silos",
            text:
              "Apps don’t share insights across platforms. Your history, preferences, and flags remain locked inside one app, even if you switch services.",
          },
        ],
      },
      {
        subtitle: "What Makes MyRoomie Different",
        content:
          "We focus on compatibility over convenience. Our matching algorithm considers lifestyle habits, conflict resolution styles, and living preferences. Onboarding takes 10 minutes but saves months of stress. Plus, we’ve implemented robust verification and safety features that other platforms lack.",
      },
      {
        subtitle: "The Road Ahead",
        content:
          "The roommate industry is ripe for disruption. By putting safety, compatibility, and user experience first, we're building the platform we wished existed when we were apartment hunting. Next, we’ll launch neighborhood-based matching and AI-driven chore scheduling.",
      },
      {
        subtitle: "Innovation Timeline",
        content:
          "• Q3 2025: Beta for AI-driven neighborhood clusters.\n• Q4 2025: Launch chore-scheduling assistant integrated with calendars.\n• Q1 2026: Mobile app redesign based on user feedback from 50,000 interviews.",
      },
      {
        subtitle: "Expert Voices",
        content: [
          {
            label: "Dr. Ellen Park, Sociologist",
            text:
              "'Values alignment is the single biggest predictor of roommate longevity. Surface preferences rarely matter after move-in.'",
          },
          {
            label: "Tom Nguyen, PropTech Analyst",
            text:
              "'MyRoomie’s layered approach to verification sets a new standard in digital trust for shared living.'",
          },
        ],
      },
      {
        subtitle: "FAQs: Platform & Performance",
        content: [
          {
            label: "Q: How fast is the average match?",
            text:
              "Users report an average of 3 days from profile completion to first meaningful conversation.",
          },
          {
            label: "Q: What’s the refund policy if I cancel?",
            text:
              "We offer a prorated refund within 7 days of subscription start—no questions asked.",
          },
        ],
      },
      {
        subtitle: "Further Studies & Links",
        content:
          "• Download our full industry report: myroomie.com/industry-report-2025\n• Case study: 'Turning Churn into Retention' — available on our blog.\n• Video panel: 'The Future of Shared Living' — recorded at PropTech Summit 2025.",
      },
    ],
  },

  4: {
    id: 4,
    title: "The Hidden Costs of Bad Roommates: A $11,000 Wake-Up Call",
    date: "2025-06-03",
    author: "Jessica Liu",
    category: "Financial Tips",
    introText:
      "From broken leases to damaged credit scores, we break down the real financial impact of poor roommate choices and how to protect yourself from costly mistakes. You’ll see real invoices, credit report screenshots, and detailed breakdowns of average expenses across 10 U.S. cities.",
    excerpt:
      "From broken leases to damaged credit scores, we break down the real financial impact of poor roommate choices and how to protect yourself. Learn how to draft airtight agreements and leverage legal tools to safeguard your wallet.",
    readTime: "7 min read",
    image:
      "https://images.unsplash.com/photo-1554224155-6726b3ff858f?w=800&h=400&fit=crop",
    likes: 156,
    comments: 12,
    featured: false,
    tags: ["Finance", "Risk Management", "Planning"],
    sections: [
      {
        subtitle: "The True Cost of Roommate Disasters",
        content: [
          {
            label: "Broken Lease Penalties",
            text:
              "Average cost: $2,000–4,000. When roommates bail unexpectedly, you're often left covering their portion or facing early termination fees. We show you amortized scenarios from NYC, LA, and Chicago.",
          },
          {
            label: "Credit Score Damage",
            text:
              "Missed rent payments can drop your credit score by 50–100 points, affecting future rental applications and loan rates for years. One case study saw a 120-point drop after two missed payments over three months.",
          },
          {
            label: "Security Deposit Loss",
            text:
              "Damage from irresponsible roommates often exceeds security deposits, leaving you with repair bills averaging $1,500–3,000. We share before-and-after apartment inspection photos to drive the point home.",
          },
          {
            label: "Legal Fees",
            text:
              "Disputes can lead to small claims court, with filing fees, attorney consultations, and lost time often costing an additional $500–1,200.",
          },
        ],
      },
      {
        subtitle: "Hidden Costs Most People Miss",
        content:
          "Beyond obvious expenses, bad roommates create indirect costs: higher utility bills from wasteful habits, replacement of damaged personal items, moving expenses when situations become unbearable, and lost productivity from constant stress and conflict. In one survey, 28% of respondents reported paying $200+ for unexpected repairs.",
      },
      {
        subtitle: "Protection Strategies",
        content:
          "Get everything in writing, including utility responsibilities and house rules. Consider renter's insurance that covers roommate situations. Screen thoroughly—a $50 background check is cheaper than a $5,000 mistake. We provide a free template lease addendum you can download.",
      },
      {
        subtitle: "When to Cut Your Losses",
        content:
          "Sometimes the best financial decision is leaving early. If your roommate consistently misses payments, damages property, or creates unsafe conditions, the cost of staying often exceeds the cost of leaving. We walk through how to negotiate early lease termination with minimal penalty.",
      },
      {
        subtitle: "Case Studies in Cost Recovery",
        content:
          "• Chicago: Tenant recovered $3,200 via small claims court after documented damage.  \n• Seattle: Two roommates split a $2,500 penalty by leveraging jointly signed addendums.  \n• Miami: User used renter’s insurance to cover $1,800 in stolen personal items.",
      },
      {
        subtitle: "FAQs & Legal Tips",
        content: [
          {
            label: "Q: Can I break my lease if my roommate defaults?",
            text:
              "Check your lease terms—some states allow early termination for roommate breach. Consult a tenant-rights attorney.",
          },
          {
            label: "Q: How do I document damage properly?",
            text:
              "Use dated photos, witness statements, and certified letters. Store everything in a secure folder.",
          },
        ],
      },
      {
        subtitle: "Resource Links",
        content:
          "• Download lease-addendum template: myroomie.com/templates/lease-addendum  \n• Tenant’s rights guide: tenantsunion.org/guide  \n• Renter’s insurance comparison chart: compareinsure.com/renter",
      },
    ],
  },

  5: {
    id: 5,
    title: "Gen Z's Housing Crisis: Why Traditional Apartment Hunting Is Broken",
    date: "2025-06-01",
    author: "David Park",
    category: "Market Analysis",
    introText:
      "Rent prices up 40%, fake listings everywhere, and platforms that don't understand modern renters. Here's what needs to change in the housing market. We analyze census data, interview Gen Z renters, and forecast trends through 2030.",
    excerpt:
      "Rent prices up 40%, fake listings everywhere, and platforms that don't understand modern renters. Here's what needs to change—and how collaborative living can save the day.",
    readTime: "10 min read",
    image:
      "https://images.unsplash.com/photo-1560518883-ce09059eeffa?w=800&h=400&fit=crop",
    likes: 298,
    comments: 63,
    featured: false,
    tags: ["Gen Z", "Housing Crisis", "Market Trends"],
    sections: [
      {
        subtitle: "The Numbers Don't Lie",
        content: [
          {
            label: "Rent Inflation",
            text:
              "Average rent has increased 40% in major cities since 2020, while entry-level salaries have grown only 12%. The math simply doesn't work for most young renters. We chart median rents versus wages to illustrate the widening gap.",
          },
          {
            label: "Fake Listing Problem",
            text:
              "Studies show 30% of rental listings on major platforms are fake or misleading, wasting renters' time and money on application fees. We share horror stories from renters who lost deposits on ghost listings.",
          },
        ],
      },
      {
        subtitle: "Why Traditional Platforms Fail Gen Z",
        content:
          "Legacy rental sites were built for a different era. They don't account for gig work income, prioritize outdated criteria, and lack the transparency and community features that younger renters expect. We break down five key feature gaps.",
      },
      {
        subtitle: "The Roommate Solution",
        content:
          "For many Gen Z renters, sharing living spaces isn't just about saving money—it's about building community and sharing resources. The platforms that succeed will understand this shift from individual to collaborative living, integrating social features and group discounts.",
      },
      {
        subtitle: "Looking Forward",
        content:
          "The housing crisis requires innovative solutions. Platforms that prioritize transparency, community, and fair matching will define the next generation of rental experiences. We outline three startup ideas poised to disrupt the industry by 2027.",
      },
      {
        subtitle: "Voices from the Community",
        content:
          "• 'I moved in with two strangers—now we split rent and host art shows.' — Zoe, 23  \n• 'I wish I'd seen a roommate-finder that asked about my remote-work schedule.' — Jamal, 27  \n• 'We form mini-co-ops for grocery runs—that’s the future.' — Priya, 22",
      },
      {
        subtitle: "Extended Data Deep Dive",
        content:
          "Download our interactive dashboard: shows rent-to-income ratios by metro areas, vacancy rates, and gig-economy income distributions. Accessible at myroomie.com/dashboards/genz.",
      },
      {
        subtitle: "Action Steps for Renters",
        content:
          "1. Build your own sub-lease clause for roommate disputes.  \n2. Partner with local co-ops to reduce living costs.  \n3. Use OurRoomie app’s group budgeting tool to track shared expenses in real time.",
      },
    ],
  },

  6: {
    id: 6,
    title: "Digital Roommate Matching: The Science Behind Compatibility",
    date: "2025-05-28",
    author: "Rachel Thompson",
    category: "Product Updates",
    introText:
      "How we've eliminated 90% of fake profiles through multi-layer verification, AI detection, and community-driven safety measures that actually work. Plus, a deep dive into our matching algorithm and the data science powering each recommendation.",
    excerpt:
      "See how AI, behavioral data, and continuous learning come together to deliver highly compatible roommate matches. We pull back the curtain on model architecture, data pipelines, and privacy safeguards.",
    readTime: "5 min read",
    image:
      "https://images.unsplash.com/photo-1563013544-824ae1b704d3?w=800&h=400&fit=crop",
    likes: 203,
    comments: 31,
    featured: false,
    tags: ["Verification", "Safety", "Technology"],
    sections: [
      {
        subtitle: "The Algorithm Behind the Magic",
        content: [
          {
            label: "Behavioral Matching",
            text:
              "We analyze communication patterns, response times, and engagement levels to identify users who are serious about finding compatible roommates. Our logistic regression model weights these signals alongside explicit preferences.",
          },
          {
            label: "Lifestyle Compatibility",
            text:
              "Our algorithm weighs factors like sleep schedules, cleanliness levels, and social preferences more heavily than surface-level interests. We normalize each input to reduce bias toward extroverts.",
          },
          {
            label: "Adaptive Weighting",
            text:
              "Over time, the model adjusts weights based on which matches lead to successful leases. We use reinforcement learning techniques to continuously optimize compatibility scores.",
          },
        ],
      },
      {
        subtitle: "Continuous Learning System",
        content:
          "Our matching algorithm improves with every successful (and unsuccessful) roommate pairing. We track long-term satisfaction through post-move surveys and adjust our compatibility models accordingly. Quarterly A/B tests validate each new feature.",
      },
      {
        subtitle: "Privacy by Design",
        content:
          "While we collect data to improve matching, user privacy is paramount. All personal information is encrypted in transit and at rest, and users control exactly what they share and with whom. We never sell data to third parties.",
      },
      {
        subtitle: "Performance Metrics",
        content:
          "• 92% reduction in false positives after adding adaptive weighting.  \n• 80% user satisfaction in post-match surveys.  \n• 1.5x increase in multi-rent lease completions year-over-year.",
      },
      {
        subtitle: "Developer Notes",
        content:
          "Our backend is built on Node.js with TypeScript, using a microservices architecture. The matching service runs in Kubernetes, with Python-based analytics pipelines in Airflow.",
      },
      {
        subtitle: "Further Reading",
        content:
          "• Blog: 'Scaling PropTech with Microservices'  \n• Paper: 'Reinforcement Learning for Real-World Matching Systems'  \n• Tutorial: 'Building Secure Verification Flows with React and MUI'.",
      },
    ],
  },

  7: {
    id: 7,
    title: "The Psychology of Shared Spaces: Creating Harmony at Home",
    date: "2025-05-25",
    author: "Dr. Emily Chen",
    category: "Tips & Guides",
    introText:
      "Understanding the psychological dynamics of shared living spaces can transform your roommate experience from stressful to supportive. Drawing on over a decade of clinical research, this article lays out the proven frameworks for cooperation, conflict resolution, and empathy-building rituals.",
    excerpt:
      "Learn why territory matters, how emotional labor affects relationships, and which positive rituals boost roommate satisfaction by 45%. Plus, free worksheet downloads to map out your living space harmony plan.",
    readTime: "9 min read",
    image:
      "https://images.unsplash.com/photo-1586023492125-27b2c045efd7?w=800&h=400&fit=crop",
    likes: 175,
    comments: 22,
    featured: false,
    tags: ["Psychology", "Living Tips", "Mental Health"],
    sections: [
      {
        subtitle: "Territory and Personal Space",
        content:
          "Humans have an innate need for personal territory. In shared spaces, clearly defined personal areas reduce conflict and increase satisfaction. This isn't just about bedrooms—it includes designated shelf space, closet areas, and even preferred seating. We provide a template for mapping out each person’s territory.",
      },
      {
        subtitle: "The Emotional Labor of Shared Living",
        content:
          "Living with others requires emotional intelligence and energy. Successful roommates recognize this invisible work and share it fairly—from remembering to buy shared items to mediating minor conflicts. Research shows equitable emotional labor shares correlate with 60% higher relationship satisfaction.",
      },
      {
        subtitle: "Building Positive Rituals",
        content:
          "Shared positive experiences create stronger bonds than simply avoiding negative ones. Weekly dinners, movie nights, or coordinated cleaning sessions can transform roommates into genuine friends. We outline 12 ritual ideas you can implement this month.",
      },
      {
        subtitle: "Conflict Resolution Frameworks",
        content:
          "• Step 1: Acknowledge feelings openly without blame.  \n• Step 2: Identify root causes behind the disagreement.  \n• Step 3: Brainstorm mutually acceptable solutions.  \n• Step 4: Document the agreement and revisit if needed.",
      },
      {
        subtitle: "Case Vignettes",
        content: [
          {
            label: "Vignette 1",
            text:
              "Two artists sharing a loft worked through noise issues by establishing 'silent hours' during paint sessions—boosting both creativity and respect.",
          },
          {
            label: "Vignette 2",
            text:
              "A remote worker and a gamer resolved schedule clashes by designating separate zones and installing lightweight soundproofing panels.",
          },
        ],
      },
      {
        subtitle: "Practical Worksheets",
        content:
          "Download our 'Shared Space Harmony' worksheet packs—PDFs with prompts for mapping territories, chore schedules, emotional check-ins, and communication logs.",
      },
    ],
  },

  8: {
    id: 8,
    title: "Remote Work and Roommate Dynamics: The New Normal",
    date: "2025-05-22",
    author: "Marcus Johnson",
    category: "Market Analysis",
    introText:
      "How remote work has fundamentally changed what people need from roommates and living spaces, and what this means for the future of co-living. We surveyed 2,000 remote workers, analyzed noise complaint data, and spoke to architects designing the next generation of home offices.",
    excerpt:
      "Discover how home offices impact roommate relationships—and which layout tweaks can boost productivity and peace. Plus, get our free floorplan guide for creating quiet zones in any apartment.",
    readTime: "8 min read",
    image:
      "https://images.unsplash.com/photo-1521737604893-d14cc227d761?w=800&h=400&fit=crop",
    likes: 134,
    comments: 19,
    featured: false,
    tags: ["Remote Work", "Lifestyle", "Future Trends"],
    sections: [
      {
        subtitle: "The Home Office Revolution",
        content:
          "With more people working from home, the apartment has become office, gym, restaurant, and social space all in one. This puts new pressures on roommate relationships and space usage. We break down three common floorplan adaptations to balance privacy with collaboration.",
      },
      {
        subtitle: "Noise and Boundary Challenges",
        content:
          "Video calls, different work schedules, and the need for quiet focus have created new sources of roommate conflict. Successful remote-work roommates establish clear communication protocols—like silent hours and signal flags—to respect each other's work boundaries.",
      },
      {
        subtitle: "The Opportunity for Connection",
        content:
          "Paradoxically, spending more time at home has also created opportunities for deeper roommate relationships. Many report feeling less isolated and more connected to their living companions than before remote work. We share five conversation prompts to deepen your connections during breaks.",
      },
      {
        subtitle: "Designing Dual-Purpose Spaces",
        content:
          "• Use modular furniture that transforms from desk to dining table.  \n• Install movable partitions for pop-up office pods.  \n• Optimize lighting for both video calls and relaxation.",
      },
      {
        subtitle: "Tools & Tech for Hybrid Living",
        content:
          "• White noise machines or ambient apps to mask distractions.  \n• Shared Google Calendar with 'Do Not Disturb' blocks.  \n• IoT light panels that signal availability to roommates.",
      },
      {
        subtitle: "Remote-Work Etiquette Guide",
        content:
          "1. Communicate your daily work hours and break times.  \n2. Agree on shared quiet hours for calls and deep work.  \n3. Use visual cues (like flags or lights) outside your door.  \n4. Plan occasional social breaks—coffee or lunch together—to maintain camaraderie.",
      },
    ],
  },
};

// Export blog posts as array for BlogList component
export const blogPosts: BlogPost[] = Object.values(blogData);

// Export categories for filtering
export const categories = [
  "All",
  "Tips & Guides",
  "Safety & Trust",
  "Industry Insights",
  "Financial Tips",
  "Market Analysis",
  "Product Updates",
  "Remote Work",
];
