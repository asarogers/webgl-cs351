import React from "react";
import {
  MapPin,
  ArrowRight,
} from "lucide-react";

import "aos/dist/aos.css";
import { motion } from "framer-motion";


export default function HeroSection(){
    return(
        <section id="home" className="scroll-mt-32 px-6 py-40 text-center bg-white min-h-screen flex items-center">
        <div className="max-w-6xl mx-auto" data-aos="fade-up">
          <h1 className="text-5xl md:text-7xl font-bold mb-6 leading-tight">
            Find your{" "}
            <span className="bg-gradient-to-r from-blue-600 via-purple-600 to-pink-600 bg-clip-text text-transparent">
              vibe
            </span>
            .<br />
            Find your{" "}
            <span className="bg-gradient-to-r from-green-600 via-teal-600 to-blue-600 bg-clip-text text-transparent">
              space
            </span>
            .<br />
            Find{" "}
            <span className="bg-gradient-to-r from-orange-600 via-red-600 to-pink-600 bg-clip-text text-transparent">
              MyRoomie
            </span>
            .
          </h1>

          <p className="text-xl md:text-2xl mb-10 text-gray-600 max-w-3xl mx-auto leading-relaxed">
            The first roommate finder that's safe, real, and actually works.
            Join thousands who've ditched fake profiles and broken filters for genuine connections.
          </p>

          <div className="flex flex-col sm:flex-row justify-center gap-4 mb-12">
            <motion.a
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
              href="https://forms.gle/qJQXtqEgHb45Y2Y8A"
              target="_blank"
              rel="noopener noreferrer"
              className="bg-black hover:bg-gray-800 text-white px-8 py-4 rounded-2xl text-lg font-semibold transition-all duration-300 flex items-center justify-center"
            >
              Become a beta tester
              <ArrowRight className="ml-2 w-5 h-5" />
            </motion.a>
            <motion.a
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
              href="#howitworks"
              className="border-2 border-black hover:bg-black hover:text-white px-8 py-4 rounded-2xl text-lg font-semibold transition-all duration-300 flex items-center justify-center"
            >
              See How It Works
              <MapPin className="ml-2 w-5 h-5" />
            </motion.a>
          </div>
          <div id="map" className="scroll-mt-28 relative max-w-4xl mx-auto">
            <div className="bg-gradient-to-r from-blue-500/20 via-purple-500/20 to-pink-500/20 rounded-3xl p-8 backdrop-blur-sm border border-white/20 shadow-2xl">
              <div className="bg-white/80 backdrop-blur rounded-2xl p-6 shadow-lg">
                <div className="flex items-center justify-between mb-4">
                  <div className="flex space-x-2">
                    <div className="w-3 h-3 bg-red-500 rounded-full"></div>
                    <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
                    <div className="w-3 h-3 bg-green-500 rounded-full"></div>
                  </div>
                  <span className="text-sm text-gray-500">
                    MyRoomie Map Demo
                  </span>
                </div>
                <div className="h-64 bg-gradient-to-br from-blue-100 via-purple-50 to-pink-100 rounded-xl flex items-center justify-center">
                  <div className="text-center">
                    <MapPin className="w-16 h-16 mx-auto mb-4 text-blue-600" />
                    <p className="text-gray-600 font-medium">
                      Interactive map demo coming soon
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>
    )
}