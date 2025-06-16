// src/components/Footer.tsx

import React from 'react';
import { Link } from 'react-router-dom';
import { HashLink } from 'react-router-hash-link';
import {
  Home,
  MapPin,
  Shield,
  Users,
  Mail,
  Twitter,
  Instagram,
} from 'lucide-react';

interface FooterProps {
  setIsOpen: (open: boolean) => void;
}

export default function Footer({ setIsOpen }: FooterProps) {
  return (
    <footer className="bg-[#BFBFBF] text-gray-800">
      {/* Top section with padding */}
      <div className="px-6 py-16 max-w-6xl mx-auto grid grid-cols-1 md:grid-cols-4 gap-12">
        {/* Brand */}
        <div className="md:col-span-1">
          <Link to="/" className="flex items-center mb-4">
            <Home className="w-6 h-6 mr-2 text-gray-900" />
            <span className="text-2xl font-bold text-gray-900">MyRoomie</span>
          </Link>
          <p className="text-gray-600 mb-4 leading-relaxed">
            The roommate finder that actually works. Safe, real, and free.
          </p>
          <div className="flex space-x-4">
            <a
              href="https://facebook.com"
              target="_blank"
              rel="noopener noreferrer"
              className="w-10 h-10 bg-black rounded-lg flex items-center justify-center text-white hover:bg-gray-800 transition-colors"
            >
              f
            </a>
            <a
              href="https://twitter.com"
              target="_blank"
              rel="noopener noreferrer"
              className="w-10 h-10 bg-black rounded-lg flex items-center justify-center text-white hover:bg-gray-800 transition-colors"
            >
              t
            </a>
            <a
              href="https://linkedin.com"
              target="_blank"
              rel="noopener noreferrer"
              className="w-10 h-10 bg-black rounded-lg flex items-center justify-center text-white hover:bg-gray-800 transition-colors"
            >
              in
            </a>
          </div>
        </div>

        {/* Product */}
        <div>
          <h5 className="font-bold mb-4 text-lg">Product</h5>
          <ul className="space-y-3">
            {[
              { label: 'Features', to: '/#features' },
              { label: 'How It Works', to: '/#howitworks' },
              { label: 'Pricing', to: '/#pricing' },
              { label: 'FAQs', to: '/#faqs' },
            ].map(({ label, to }) => (
              <li key={label}>
                <HashLink
                  smooth
                  to={to}
                  className="text-gray-600 hover:text-gray-900 transition-colors"
                >
                  {label}
                </HashLink>
              </li>
            ))}
          </ul>
        </div>

        {/* Trust & Safety */}
        <div>
          <h5 className="font-bold mb-4 text-lg">Trust &amp; Safety</h5>
          <ul className="space-y-3">
            {[
              { label: 'Verification Tiers', to: '/#verificationtiers' },
              { label: 'Community Guidelines', to: '/#communityguidelines' },
              { label: 'Report a Profile', to: '/#reportaprofile' },
              { label: 'Safety Tips', to: '/#safetytips' },
            ].map(({ label, to }) => (
              <li key={label}>
                <HashLink
                  smooth
                  to={to}
                  className="text-gray-600 hover:text-gray-900 transition-colors"
                >
                  {label}
                </HashLink>
              </li>
            ))}
          </ul>
        </div>

        {/* Company */}
        <div>
          <h5 className="font-bold mb-4 text-lg">Company</h5>
          <ul className="space-y-3">
            <li>
              <button
                onClick={() => setIsOpen(true)}
                className="text-gray-700 hover:text-gray-900 font-medium transition-colors"
              >
                Contact Us
              </button>
            </li>
            {[
              { label: 'About Us', to: '/#aboutus', hash: true },
              { label: 'Careers', to: '/careers' },
              { label: 'Terms & Privacy', to: '/terms' },
            ].map(({ label, to, hash }) => (
              <li key={label}>
                {hash ? (
                  <HashLink
                    smooth
                    to={to}
                    className="text-gray-600 hover:text-gray-900 transition-colors"
                  >
                    {label}
                  </HashLink>
                ) : (
                  <Link
                    to={to}
                    className="text-gray-600 hover:text-gray-900 transition-colors"
                  >
                    {label}
                  </Link>
                )}
              </li>
            ))}
          </ul>
        </div>
      </div>

      {/* Bottom copyright & policies */}
      <div className="border-t border-gray-400 py-8">
        <div className="max-w-6xl mx-auto flex flex-col md:flex-row justify-between items-center px-6">
          <p className="text-gray-600 mb-4 md:mb-0">
            © 2025 MyRoomie. All rights reserved.
          </p>
          <div className="flex space-x-6 text-sm">
            <Link
              to="/privacy"
              className="text-gray-600 hover:text-gray-900 transition-colors"
            >
              Privacy Policy
            </Link>
            <Link
              to="/terms"
              className="text-gray-600 hover:text-gray-900 transition-colors"
            >
              Terms of Service
            </Link>
            <Link
              to="/cookie-policy"
              className="text-gray-600 hover:text-gray-900 transition-colors"
            >
              Cookie Policy
            </Link>
          </div>
        </div>
      </div>
    </footer>
  );
}
