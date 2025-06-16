import { useState } from "react";
import "aos/dist/aos.css";
import { motion } from "framer-motion";
import {
  features,
} from "./Data.tsx"; 

export default function Features() {
  const [activeFeature, setActiveFeature] = useState(0);

  const nextFeature = () => {
    setActiveFeature((prev) => (prev + 1) % features.length);
  };

  const prevFeature = () => {
    setActiveFeature((prev) => (prev - 1 + features.length) % features.length);
  };

  return (
    <section id="features" data-aos="fade-up" className="scroll-mt-20 px-6 py-20 bg-white">
    <div className="max-w-6xl mx-auto">
      <div className="text-center mb-16">
        <h2 className="text-4xl md:text-5xl font-bold mb-6">
          Features You'll Actually Use
        </h2>
        <p className="text-xl text-gray-600 max-w-2xl mx-auto">
          Built by roommate searchers, for roommate searchers. Every feature solves a real problem.
        </p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
        {features.map((feature, index) => (
          <motion.div
            key={index}
            data-aos="zoom-in-up"
            whileHover={{ scale: 1.03 }}
            className="group relative overflow-hidden bg-gradient-to-br from-[#FDFBF7] to-white p-8 rounded-2xl border-2 border-gray-200 hover:border-transparent transition-all duration-300 transform hover:-translate-y-2 hover:shadow-2xl"
            onMouseEnter={() => setHoveredFeature(index)}
            onMouseLeave={() => setHoveredFeature(null)}
          >
            <div className={`absolute inset-0 bg-gradient-to-br ${feature.color} opacity-0 group-hover:opacity-10 transition-opacity duration-300`}></div>

            <div className="relative z-10">
              <div className={`inline-flex p-4 bg-gradient-to-br ${feature.color} rounded-2xl text-white mb-6 group-hover:scale-110 transition-transform duration-300`}>
                {feature.icon}
              </div>

              <h3 className="text-xl font-bold mb-4 group-hover:text-gray-800 transition-colors">
                {feature.title}
              </h3>

              <p className="text-gray-600 leading-relaxed group-hover:text-gray-700 transition-colors">
                {feature.description}
              </p>
            </div>
          </motion.div>
        ))}
      </div>
    </div>
  </section>
  );
}