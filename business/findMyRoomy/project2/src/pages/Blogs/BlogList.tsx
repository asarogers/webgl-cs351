// src/pages/Blogs/BlogList.tsx

import React, { useState } from "react";
import {
  Calendar,
  Clock,
  ArrowRight,
  User,
  Heart,
  MessageCircle,
  TrendingUp,
} from "lucide-react";
import { Link } from "react-router-dom";
import { blogPosts, categories } from "./data/blogData";
import Navbar from "../../components/Navbar";
import Footer from "../../components/Footer";
import ContactUs from "../../components/ContactUs";


export default function BlogList({setIsOpen, isOpen}) {
  const [selectedCategory, setSelectedCategory] = useState("All");
  const [searchTerm, setSearchTerm] = useState("");

  const filteredPosts = blogPosts.filter((post) => {
    const matchesCategory =
      selectedCategory === "All" || post.category === selectedCategory;
    const matchesSearch =
      post.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
      post.excerpt.toLowerCase().includes(searchTerm.toLowerCase()) ||
      post.tags.some((tag) =>
        tag.toLowerCase().includes(searchTerm.toLowerCase())
      );
    return matchesCategory && matchesSearch;
  });

  const featuredPosts = filteredPosts.filter((post) => post.featured);
  const regularPosts = filteredPosts.filter((post) => !post.featured);

  return (
    <div className="flex flex-col min-h-screen bg-gradient-to-br from-[#FDFBF7] to-white">
      {/* Navbar stays at top */}
        {isOpen && <ContactUs onClose={() => setIsOpen(false)} />}
      <Navbar setIsOpen={setIsOpen}/>

      {/* Main content grows to fill available space */}
      <main className="flex-grow px-6 py-20">
        <div className="max-w-7xl mx-auto">
          {/* Header */}
          <div className="text-center mb-16">
            <div className="inline-flex items-center bg-gradient-to-r from-blue-100 to-purple-100 px-4 py-2 rounded-full mb-6">
              <TrendingUp className="w-4 h-4 mr-2 text-blue-600" />
              <span className="text-sm font-semibold text-blue-800">
                Latest Insights
              </span>
            </div>
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              The MyRoomie Blog
            </h2>
            <p className="text-xl text-gray-600 max-w-2xl mx-auto">
              Real insights, practical tips, and industry analysis from
              roommate experts who've been there.
            </p>
          </div>

          {/* Filters */}
          <div className="mb-12 flex flex-col md:flex-row gap-6 items-center justify-between">
            <div className="relative flex-1 max-w-md">
              <input
                type="text"
                placeholder="Search articles..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="w-full px-4 py-3 pl-12 bg-white border-2 border-gray-200 rounded-xl focus:border-blue-500 focus:outline-none transition-all duration-200"
              />
              <div className="absolute left-4 top-1/2 transform -translate-y-1/2 text-gray-400">
                🔍
              </div>
            </div>

            <div className="flex flex-wrap gap-2">
              {categories.map((category) => (
                <button
                  key={category}
                  onClick={() => setSelectedCategory(category)}
                  className={`px-4 py-2 rounded-xl font-medium transition-all duration-200 ${
                    selectedCategory === category
                      ? "bg-black text-white shadow-lg"
                      : "bg-white text-gray-700 hover:bg-gray-100 border border-gray-200"
                  }`}
                >
                  {category}
                </button>
              ))}
            </div>
          </div>

          {/* Featured Posts */}
          {featuredPosts.length > 0 && (
            <div className="mb-16">
              <h3 className="text-2xl font-bold mb-8 flex items-center">
                <span className="bg-gradient-to-r from-orange-500 to-red-500 text-white px-3 py-1 rounded-lg text-sm mr-3">
                  Featured
                </span>
                Must-Read Articles
              </h3>
              <div className="grid md:grid-cols-2 gap-8">
                {featuredPosts.map((post) => (
                  <article
                    key={post.id}
                    className="group bg-white rounded-2xl shadow-lg hover:shadow-2xl transition-all duration-300 transform hover:-translate-y-2 overflow-hidden border border-gray-100"
                  >
                    <Link to={`/blog/${post.id}`}>
                      <div className="relative overflow-hidden">
                        <img
                          src={post.image}
                          alt={post.title}
                          className="w-full h-64 object-cover group-hover:scale-105 transition-transform duration-500"
                        />
                        <div className="absolute top-4 left-4">
                          <span className="bg-black text-white px-3 py-1 rounded-full text-sm font-medium">
                            {post.category}
                          </span>
                        </div>
                      </div>

                      <div className="p-8">
                        <div className="flex items-center text-sm text-gray-500 mb-4">
                          <User className="w-4 h-4 mr-2" />
                          <span className="mr-4">{post.author}</span>
                          <Calendar className="w-4 h-4 mr-2" />
                          <span className="mr-4">
                            {new Date(post.date).toLocaleDateString()}
                          </span>
                          <Clock className="w-4 h-4 mr-2" />
                          <span>{post.readTime}</span>
                        </div>

                        <h3 className="text-xl font-bold mb-3 group-hover:text-blue-600 transition-colors line-clamp-2">
                          {post.title}
                        </h3>

                        <p className="text-gray-600 mb-4 line-clamp-3">
                          {post.excerpt}
                        </p>

                        <div className="flex items-center space-x-4 text-sm text-gray-500">
                          <div className="flex items-center">
                            <Heart className="w-4 h-4 mr-1" />
                            <span>{post.likes}</span>
                          </div>
                          <div className="flex items-center">
                            <MessageCircle className="w-4 h-4 mr-1" />
                            <span>{post.comments}</span>
                          </div>
                        </div>
                      </div>
                    </Link>
                  </article>
                ))}
              </div>
            </div>
          )}

          {/* Regular Posts */}
          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8">
            {regularPosts.map((post) => (
              <article
                key={post.id}
                className="group bg-white rounded-2xl shadow-lg hover:shadow-xl transition-all duration-300 transform hover:-translate-y-1 overflow-hidden border border-gray-100"
              >
                <Link to={`/blog/${post.id}`}>
                  <div className="relative overflow-hidden">
                    <img
                      src={post.image}
                      alt={post.title}
                      className="w-full h-48 object-cover group-hover:scale-105 transition-transform duration-500"
                    />
                    <div className="absolute top-3 left-3">
                      <span className="bg-white/90 backdrop-blur text-gray-800 px-3 py-1 rounded-full text-xs font-medium">
                        {post.category}
                      </span>
                    </div>
                  </div>

                  <div className="p-6">
                    <div className="flex items-center text-xs text-gray-500 mb-3">
                      <span className="mr-3">{post.author}</span>
                      <span className="mr-3">•</span>
                      <span className="mr-3">
                        {new Date(post.date).toLocaleDateString()}
                      </span>
                      <span className="mr-3">•</span>
                      <span>{post.readTime}</span>
                    </div>

                    <h3 className="text-lg font-bold mb-3 group-hover:text-blue-600 transition-colors line-clamp-2">
                      {post.title}
                    </h3>

                    <p className="text-gray-600 text-sm mb-4 line-clamp-3">
                      {post.excerpt}
                    </p>

                    <div className="flex flex-wrap gap-2 mb-4">
                      {post.tags.slice(0, 2).map((tag) => (
                        <span
                          key={tag}
                          className="bg-gray-100 text-gray-600 px-2 py-1 rounded-lg text-xs"
                        >
                          #{tag}
                        </span>
                      ))}
                    </div>

                    <div className="flex items-center space-x-3 text-xs text-gray-500">
                      <div className="flex items-center">
                        <Heart className="w-3 h-3 mr-1" />
                        <span>{post.likes}</span>
                      </div>
                      <div className="flex items-center">
                        <MessageCircle className="w-3 h-3 mr-1" />
                        <span>{post.comments}</span>
                      </div>
                    </div>
                  </div>
                </Link>
              </article>
            ))}
          </div>

          {/* No Results */}
          {filteredPosts.length === 0 && (
            <div className="text-center py-12">
              <div className="text-6xl mb-4">📝</div>
              <h3 className="text-xl font-semibold text-gray-700 mb-2">
                No articles found
              </h3>
              <p className="text-gray-500">
                Try adjusting your search terms or selected category.
              </p>
            </div>
          )}
        </div>
      </main>

      {/* Footer at the very bottom, spans full width */}
      <div className="w-full">
        <Footer setIsOpen ={setIsOpen}/>
      </div>
    </div>
  );
}
