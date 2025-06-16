import React, { useState, useEffect } from "react";
import {
  MapPin,
  Shield,
  Filter,
  MessageCircle,
  RefreshCw,
  Users,
  Home,
  Zap,
  CheckCircle,
  ArrowRight,
  Star,
  Clock,
  Search,
  Award,
  TrendingUp,
} from "lucide-react";

export default function Safety() {
  return (
    <section className="px-6 py-20 bg-white">
      <div className="max-w-6xl mx-auto">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold mb-6">
            Built for Safety & Trust
          </h2>
          <p className="text-xl text-gray-600 max-w-2xl mx-auto">
            Your safety is our priority. Multiple verification layers ensure
            authentic connections.
          </p>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          <div className="text-center p-8 bg-gradient-to-br from-blue-50 to-cyan-50 rounded-2xl">
            <Shield className="w-16 h-16 mx-auto mb-6 text-blue-600" />
            <h3 className="text-xl font-bold mb-4">Identity Verification</h3>
            <p className="text-gray-600">
              Government ID verification and background checks for all users.
            </p>
          </div>

          <div className="text-center p-8 bg-gradient-to-br from-green-50 to-teal-50 rounded-2xl">
            <Award className="w-16 h-16 mx-auto mb-6 text-green-600" />
            <h3 className="text-xl font-bold mb-4">Community Standards</h3>
            <p className="text-gray-600">
              Strict community guidelines with 24/7 moderation and reporting.
            </p>
          </div>

          <div className="text-center p-8 bg-gradient-to-br from-purple-50 to-pink-50 rounded-2xl">
            <Users className="w-16 h-16 mx-auto mb-6 text-purple-600" />
            <h3 className="text-xl font-bold mb-4">Smart Matching</h3>
            <p className="text-gray-600">
              AI-powered compatibility scoring prevents mismatched situations.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}