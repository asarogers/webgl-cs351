import React, { useState, useEffect } from "react";
import ContactUs from "../components/ContactUs.tsx";
import AOS from "aos";
import "aos/dist/aos.css";

import Safety from "../components/Safety.tsx";
import Navbar from "../components/Navbar.tsx";
import Footer from "../components/Footer.tsx";
import PainPoints from "../components/PainPoints.tsx";
import HeroSection from "../components/HeroSection.tsx";
import Features from "../components/Features.tsx";
import HowItWorks from "../components/HowItWorks.tsx";
import CTA from "../components/CTA.tsx";
import Guarantees from "../components/Guarantees.tsx";

export default function LandingPage({setIsOpen,isOpen}) {


  useEffect(() => {
    AOS.init({ duration: 800 });
  }, []);
  return (
    <div className="bg-[#FDFBF7] text-gray-900 font-sans overflow-x-hidden">
      {isOpen && <ContactUs onClose={() => setIsOpen(false)} />}
        < Navbar setIsOpen={setIsOpen}/>
      
     <HeroSection />
     < PainPoints />
     <Features />
      < HowItWorks />

      {/* Social Proof Section */}
      {/* <section className="px-6 py-20 bg-white">
        <div className="max-w-6xl mx-auto">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Real Stories from Real Users</h2>
            <p className="text-xl text-gray-600 max-w-2xl mx-auto">
              See why thousands choose MyRoomie over other platforms.
            </p>
          </div>
          
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            {testimonials.map((testimonial, index) => (
              <div key={index} className="bg-gradient-to-br from-[#FDFBF7] to-white p-8 rounded-2xl shadow-lg border border-gray-100 hover:shadow-xl transition-all duration-300">
                <div className="flex mb-4">
                  {[...Array(testimonial.rating)].map((_, i) => (
                    <Star key={i} className="w-5 h-5 text-yellow-500 fill-current" />
                  ))}
                </div>
                <p className="text-lg font-medium mb-6 italic text-gray-700">"{testimonial.quote}"</p>
                <div className="border-t pt-4">
                  <div className="font-bold text-gray-900">{testimonial.name}</div>
                  <div className="text-gray-600">{testimonial.role}</div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </section> */}

      <Safety />
      < Guarantees />
      < CTA />
     < Footer setIsOpen={setIsOpen}/>
    </div>
  );
}