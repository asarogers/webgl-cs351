import React, { useState, useEffect } from "react";
import "aos/dist/aos.css";
import { motion } from "framer-motion";
import {
  steps,
} from "./Data.tsx"; 

export default function HowItWorks() {
  return (
  <section id="howitworks" data-aos="fade-up" className="scroll-mt-24 px-6 py-20 bg-gradient-to-br from-[#E4DDD2] to-[#D6CFC4]">
        <div className="max-w-6xl mx-auto">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              How It Works
            </h2>
            <p className="text-xl text-gray-600 max-w-2xl mx-auto">
              Three simple steps to find your perfect roommate and apartment match.
            </p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-12">
            {steps.map((step, index) => (
              <motion.div
                key={index}
                whileHover={{ scale: 1.03 }}
                className="text-center group"
                data-aos="fade-up"
                data-aos-delay={index * 150}
              >
                <div className="relative mb-8">
                  <div className="w-24 h-24 bg-gradient-to-br from-blue-500 to-purple-600 rounded-full flex items-center justify-center mx-auto mb-4 text-white group-hover:scale-110 transition-transform duration-300 shadow-2xl">
                    {step.icon}
                  </div>
                  <div className="absolute -top-2 -right-2 w-8 h-8 bg-white rounded-full flex items-center justify-center shadow-lg border-2 border-gray-200 text-sm font-bold text-gray-800">
                    {index + 1}
                  </div>
                </div>

                <h4 className="text-xl font-bold mb-4 group-hover:text-blue-600 transition-colors">
                  {step.title}
                </h4>
                <p className="text-gray-600 leading-relaxed">{step.description}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>
  );
}