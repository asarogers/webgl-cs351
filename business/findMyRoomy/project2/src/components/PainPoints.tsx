import React, { useEffect } from "react";
import AOS from "aos";
import "aos/dist/aos.css";
import { motion } from "framer-motion";
import { painPoints } from "./Data.tsx";

export default function PainPoints() {
  useEffect(() => {
    AOS.init({ duration: 800 });
  }, []);
  return (
    <section
      data-aos="fade-up"
      className="px-6 py-20 bg-gradient-to-br from-[#E4DDD2] to-[#D6CFC4]"
    >
      <div className="max-w-6xl mx-auto">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold mb-6">
            What's broken — and how we fix it
          </h2>
          <p className="text-xl text-gray-600 max-w-2xl mx-auto">
            The average person wastes $11,000+ and 6 months searching for
            roommates. Here's how we change that.
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
          {painPoints.map((point, index) => (
            <motion.div
              key={index}
              whileHover={{ scale: 1.02 }}
              className="group bg-white p-8 rounded-2xl shadow-lg hover:shadow-2xl transition-all duration-300 transform hover:-translate-y-2 border border-gray-100"
              data-aos="fade-up"
              data-aos-delay={index * 100}
            >
              <div className="flex items-start space-x-4">
                <div
                  className={`p-3 rounded-xl bg-gradient-to-br ${
                    point.color === "text-red-600"
                      ? "from-red-100 to-red-200"
                      : point.color === "text-orange-600"
                      ? "from-orange-100 to-orange-200"
                      : point.color === "text-purple-600"
                      ? "from-purple-100 to-purple-200"
                      : "from-blue-100 to-blue-200"
                  }`}
                >
                  {point.icon}
                </div>
                <div className="flex-1">
                  <h3
                    className={`text-xl font-bold mb-2 ${point.color} group-hover:scale-105 transition-transform`}
                  >
                    {point.problem}
                  </h3>
                  <p className="text-gray-700 leading-relaxed mb-3">
                    <span className="text-green-600 font-semibold">
                      Our solution:
                    </span>{" "}
                    {point.solution}
                  </p>
                  <div className="text-sm text-gray-500 font-medium bg-gray-50 px-3 py-1 rounded-full inline-block">
                    💰 {point.impact}
                  </div>
                </div>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}
