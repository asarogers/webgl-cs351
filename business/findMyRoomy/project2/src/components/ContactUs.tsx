import React from "react";
import { MapPin, Phone, Mail, X, Facebook, Linkedin, Youtube } from "lucide-react";
import emailjs from "emailjs-com";

export default function ContactUs({ onClose }) {
  const sendEmail = (e) => {
    e.preventDefault();

    emailjs
      .sendForm(
        "service_x9x3x5e",          // ✅ your service ID
        "template_g1qck6c",         // ✅ your template ID
        e.target,
        "4D_p0Cy6YqcSa4sPU"     // 🔁 replace this with your actual public key
      )
      .then(
        (result) => {
          alert("✅ Message sent successfully!");
          e.target.reset();
          onClose();
        },
        (error) => {
          console.error("❌ Error sending email:", error.text);
          alert("Something went wrong. Please try again.");
        }
      );
  };

  return (
    <div className="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center">
      <div className="bg-white max-w-4xl w-full mx-4 md:mx-0 rounded-2xl shadow-2xl overflow-hidden relative animate-fade-in">
        <button
          onClick={onClose}
          className="absolute top-4 right-4 text-gray-400 hover:text-gray-600 hover:bg-gray-100 p-2 rounded-full transition-all duration-200 z-10"
        >
          <X className="w-5 h-5" />
        </button>

        <div className="grid md:grid-cols-2">
          {/* Left - Form */}
          <div className="p-8 bg-gradient-to-br from-slate-50 via-blue-50 to-indigo-50">
            <h2 className="text-3xl font-bold text-slate-800 mb-6">Get In Touch</h2>
            <form className="space-y-5" onSubmit={sendEmail}>
              <input
                type="text"
                name="name"
                placeholder="Your Name"
                required
                className="w-full p-4 border border-black rounded-xl bg-white/80 text-slate-700 placeholder-slate-400 focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-all duration-200"
              />
              <input
                type="email"
                name="email"
                placeholder="youremail@mail.com"
                required
                className="w-full p-4 border border-black rounded-xl bg-white/80 text-slate-700 placeholder-slate-400 focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-all duration-200"
              />
              <textarea
                name="message"
                placeholder="Type your message..."
                required
                className="w-full p-4 h-32 border border-black rounded-xl bg-white/80 text-slate-700 placeholder-slate-400 focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-all duration-200 resize-none"
              ></textarea>
              <button
                type="submit"
                className="w-full py-4 bg-gradient-to-r from-blue-600 via-indigo-600 to-purple-600 text-white font-semibold rounded-xl hover:from-blue-700 hover:via-indigo-700 hover:to-purple-700 transform hover:scale-[1.02] transition-all duration-300 shadow-lg hover:shadow-xl"
              >
                Send Message
              </button>
            </form>
          </div>

          {/* Right - Contact Info */}
          <div className="bg-gradient-to-br from-slate-800 via-slate-700 to-slate-900 p-8 flex flex-col justify-center text-white">
            <h3 className="text-3xl font-bold mb-4">Contact Information</h3>
            <p className="text-slate-300 mb-8 text-lg leading-relaxed">
              Have a question, want to connect, or work together?
            </p>

            <div className="space-y-6">
              <div className="flex items-center space-x-4">
                <div className="bg-blue-500 p-3 rounded-full">
                  <MapPin className="w-5 h-5 text-white" />
                </div>
                <span className="text-slate-200 text-lg">Chicago, IL</span>
              </div>
              <div className="flex items-center space-x-4">
                <div className="bg-green-500 p-3 rounded-full">
                  <Phone className="w-5 h-5 text-white" />
                </div>
                <span className="text-slate-200 text-lg">000-000-0000</span>
              </div>
              <div className="flex items-center space-x-4">
                <div className="bg-purple-500 p-3 rounded-full">
                  <Mail className="w-5 h-5 text-white" />
                </div>
                <span className="text-slate-200 text-lg">cryptocodesace@gmail.com</span>
              </div>
            </div>

            <div className="mt-8">
              <div className="text-slate-300 mb-4 text-lg">Follow us on:</div>
              <div className="flex space-x-4">
                <div className="bg-blue-600 hover:bg-blue-700 p-3 rounded-full cursor-pointer transition-all duration-200 hover:scale-110">
                  <Facebook className="w-5 h-5 text-white" />
                </div>
                <div className="bg-blue-700 hover:bg-blue-800 p-3 rounded-full cursor-pointer transition-all duration-200 hover:scale-110">
                  <Linkedin className="w-5 h-5 text-white" />
                </div>
                <div className="bg-red-600 hover:bg-red-700 p-3 rounded-full cursor-pointer transition-all duration-200 hover:scale-110">
                  <Youtube className="w-5 h-5 text-white" />
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
