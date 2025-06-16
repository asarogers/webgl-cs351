// src/components/Navbar.tsx

import React, { useState } from "react";
import { Link } from "react-router-dom";
import { HashLink } from "react-router-hash-link";
import { Home, TrendingUp, Star, Users, Clock, X } from "lucide-react";
import "aos/dist/aos.css";
import { motion } from "framer-motion";

interface NavbarProps {
  setIsOpen: (open: boolean) => void;
}

export default function Navbar({ setIsOpen }: NavbarProps) {
  const [trustBarVisible, setTrustBarVisible] = useState(true);
  const sections = ["Home", "Features", "How It Works"];

  return (
    <>
      <header className="bg-white shadow-sm fixed top-0 left-0 w-full z-50">
        <div className="max-w-6xl mx-auto flex items-center justify-between px-6 py-4">
          {/* Logo */}
          <div className="flex items-center">
            <Home className="w-6 h-6 text-black mr-2" />
            <span className="text-xl font-bold text-gray-900">MyRoomie</span>
          </div>

          {/* Navigation */}
          <nav>
            <ul className="flex space-x-8 text-gray-700 items-center">
              {sections.map((item) => {
                const hash = item.replace(/\s+/g, "").toLowerCase();
                return (
                  <li key={hash}>
                    <HashLink
                      smooth
                      to={`/#${hash}`}
                      className="text-gray-700 hover:text-black font-medium"
                    >
                      {item}
                    </HashLink>
                  </li>
                );
              })}

              {/* Blogs link now navigates to /blog */}
              <li>
                <Link
                  to="/blog"
                  className="text-gray-700 hover:text-black font-medium"
                >
                  Blogs
                </Link>
              </li>

              {/* Contact button */}
              <li>
                <motion.button
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.97 }}
                  onClick={() => setIsOpen(true)}
                  className="bg-gradient-to-r from-blue-600 via-purple-600 to-pink-600 text-white px-6 py-2 rounded-lg font-semibold hover:brightness-110 transition-all"
                >
                  Contact Us
                </motion.button>
              </li>
            </ul>
          </nav>
        </div>
      </header>

      {/* Trust bar */}
      {trustBarVisible && (
        <div className="fixed top-[72px] left-0 w-full bg-gradient-to-r from-blue-800 via-cyan-700 to-gray-800 text-white px-6 py-3 text-center z-40">
          <div className="relative max-w-6xl mx-auto">
            <div className="flex items-center justify-center space-x-8 text-sm">
              <div className="flex items-center space-x-2">
                <TrendingUp className="w-4 h-4" />
                <span>Our Goal</span>
              </div>
              <div className="flex items-center space-x-2">
                <Users className="w-4 h-4" />
                <span>10,000+ verified users</span>
              </div>
              <div className="flex items-center space-x-2">
                <Star className="w-4 h-4" />
                <span>4.9/5 rating</span>
              </div>
              <div className="flex items-center space-x-2">
                <Clock className="w-4 h-4" />
                <span>7-day avg. match time</span>
              </div>
            </div>
            <button
              onClick={() => setTrustBarVisible(false)}
              className="absolute right-0 top-1/2 transform -translate-y-1/2 hover:bg-white/20 rounded-full p-1 transition-colors"
              aria-label="Close trust bar"
            >
              <X className="w-4 h-4" />
            </button>
          </div>
        </div>
      )}

      {/* Spacer for fixed header + trust bar */}
      <div className={trustBarVisible ? "mt-[120px]" : "mt-[72px]"} />
    </>
  );
}
