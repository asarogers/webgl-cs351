import {
    MapPin,
    Users,
    Home,
    Zap,
    Search,
    CheckCircle,
    Shield,
    Filter,
    MessageCircle,
    RefreshCw,
  } from "lucide-react";
  
  // Features Section
  export const features = [
    {
      icon: <MapPin className="w-8 h-8" />,
      title: "Draw Your Zone",
      description:
        "Select your exact living area using intuitive touch gestures on our interactive map.",
      color: "from-blue-500 to-cyan-500",
    },
    {
      icon: <Users className="w-8 h-8" />,
      title: "Match by Vibe",
      description:
        "Find compatible roommates based on lifestyle preferences, values, and daily routines.",
      color: "from-purple-500 to-pink-500",
    },
    {
      icon: <Home className="w-8 h-8" />,
      title: "Group Applications",
      description:
        "Apply together and split costs easily with built-in group coordination tools.",
      color: "from-green-500 to-teal-500",
    },
    {
      icon: <Zap className="w-8 h-8" />,
      title: "Instant Matching",
      description:
        "Get matched instantly with AI-powered compatibility scoring and real-time notifications.",
      color: "from-orange-500 to-red-500",
    },
  ];
  
  // How It Works Steps
  export const steps = [
    {
      icon: <Search className="w-12 h-12" />,
      title: "Draw Your Zone",
      description:
        "Use our interactive map to draw your ideal living area and set your preferences.",
    },
    {
      icon: <Users className="w-12 h-12" />,
      title: "Get Matched",
      description:
        "Our AI connects you with compatible roommates and available apartments in your zone.",
    },
    {
      icon: <CheckCircle className="w-12 h-12" />,
      title: "Apply Together",
      description:
        "Submit applications in one click, alone or with your matched group.",
    },
  ];
  
  // Pain Points Section
  export const painPoints = [
    {
      icon: <Shield className="w-6 h-6" />,
      problem: "90% Fake Profiles",
      solution: "Verified users & background checks keep it real",
      color: "text-red-600",
      impact: "Save $2,400 avg. scam loss",
    },
    {
      icon: <Filter className="w-6 h-6" />,
      problem: "Filters Don't Work",
      solution: "AI-powered filtering & dealbreaker logic that actually works",
      color: "text-orange-600",
      impact: "Save 120+ hours searching",
    },
    {
      icon: <MessageCircle className="w-6 h-6" />,
      problem: "Paywalls to Message",
      solution: "Always-free matching & messaging for everyone",
      color: "text-purple-600",
      impact: "Save $360/year in fees",
    },
    {
      icon: <RefreshCw className="w-6 h-6" />,
      problem: "Outdated Listings",
      solution: "Real-time partner apartment inventory that's always fresh",
      color: "text-blue-600",
      impact: "Avoid dead-end searches",
    },
  ];
  
  // Testimonials Section
  export const testimonials = [
    {
      name: "Sarah M.",
      role: "Marketing Manager, Chicago",
      quote:
        "Found my perfect roommate in just 3 days after months of searching elsewhere. The AI matching is incredibly accurate.",
      rating: 5,
    },
    {
      name: "Mike R.",
      role: "Software Engineer, Austin",
      quote:
        "The group application feature saved us so much time. We applied to 12 places in one afternoon and got approved for our top choice.",
      rating: 5,
    },
    {
      name: "Jessica L.",
      role: "Graduate Student, Boston",
      quote:
        "I was skeptical about online roommate finding, but the verification process here made me feel completely safe.",
      rating: 5,
    },
  ];
  
  // Guarantees Section
  export const guarantees = [
    {
      icon: <CheckCircle className="w-6 h-6 text-green-600" />,
      title: "7-Day Match Promise",
      description:
        "Find compatible matches within a week, or we'll personally help you find options.",
    },
    {
      icon: <Shield className="w-6 h-6 text-blue-600" />,
      title: "100% Verified Profiles",
      description:
        "Every user goes through identity verification and background screening.",
    },
    {
      icon: <MessageCircle className="w-6 h-6 text-purple-600" />,
      title: "Always Free Core Features",
      description:
        "Matching, messaging, and basic features remain free forever.",
    },
  ];
  