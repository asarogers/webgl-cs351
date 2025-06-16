import React from 'react';
import { Star, Quote } from 'lucide-react';

const testimonials = [
  {
    name: "Sarah M.",
    age: 24,
    location: "Austin, TX",
    quote: "Finally a roommate app that doesn't feel sketchy. The verification system actually works and I found my perfect roommate in 2 weeks!",
    rating: 5,
  },
  {
    name: "Jake L.",
    age: 27,
    location: "Denver, CO",
    quote: "Being able to draw my zone and find real listings is a game changer. No more fake apartments or broken location filters.",
    rating: 5,
  },
  {
    name: "Maya P.",
    age: 22,
    location: "Seattle, WA",
    quote: "I used to be scared of scams—this felt like Airbnb + Hinge for roommates. The vibe matching actually works!",
    rating: 5,
  },
];

export default function Testimonials() {
  return (
    <section className="py-20 bg-gradient-to-b from-gray-50 to-white">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="text-center mb-16">
          <h2 className="text-4xl font-bold text-gray-900 mb-4">
            What our early users are saying
          </h2>
          <p className="text-xl text-gray-600">
            Real stories from real people who found their perfect roommates
          </p>
        </div>

        <div className="grid md:grid-cols-3 gap-8">
          {testimonials.map((testimonial, index) => (
            <div
              key={index}
              className="bg-white rounded-2xl p-8 shadow-lg hover:shadow-xl transition-shadow relative"
            >
              <Quote className="w-8 h-8 text-teal-500 mb-4" />
              
              <div className="flex mb-4">
                {[...Array(testimonial.rating)].map((_, i) => (
                  <Star key={i} className="w-5 h-5 text-yellow-400 fill-current" />
                ))}
              </div>

              <blockquote className="text-gray-700 leading-relaxed mb-6">
                "{testimonial.quote}"
              </blockquote>

              <div className="flex items-center">
                <div className="w-12 h-12 bg-gradient-to-r from-teal-400 to-cyan-400 rounded-full flex items-center justify-center mr-4">
                  <span className="text-white font-bold text-lg">
                    {testimonial.name.charAt(0)}
                  </span>
                </div>
                <div>
                  <p className="font-semibold text-gray-900">{testimonial.name}</p>
                  <p className="text-sm text-gray-500">
                    Age {testimonial.age} • {testimonial.location}
                  </p>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}