// src/pages/SingleBlog.tsx

import React, { useState } from "react";
import {
  Box,
  Typography,
  Container,
  Avatar,
  Chip,
  Divider,
  Button,
  Snackbar,
  Alert,
} from "@mui/material";
import { useParams, useNavigate, Link } from "react-router-dom";
import {
  Calendar,
  Clock,
  ArrowLeft,
  Share2,
  Bookmark,
} from "lucide-react";
import BlogSidebar from "../../components/BlogSidebar";
import { blogData } from "./data/blogData";
import Footer from "../../components/Footer";
import ContactUs from "../../components/ContactUs";


export default function SingleBlog({setIsOpen, isOpen}) {
  const { id } = useParams<{ id: string }>();
  const navigate = useNavigate();
  const blog = blogData[Number(id)];

  // Snackbars for share & bookmark fallbacks
  const [shareSnackbarOpen, setShareSnackbarOpen] = useState(false);
  const [bookmarkSnackbarOpen, setBookmarkSnackbarOpen] = useState(false);

  if (!blog) {
    return (
      <div className="min-h-screen bg-gray-50 flex items-center justify-center">
        {isOpen && <ContactUs onClose={() => setIsOpen(false)} />}
        <div className="text-center">
          <div className="text-6xl mb-4">📝</div>
          <Typography variant="h5" color="error" className="mb-4">
            Blog post not found.
          </Typography>
          <Button
            onClick={() => navigate("/blog")}
            variant="contained"
            className="bg-blue-600 hover:bg-blue-700"
          >
            Return to Blog
          </Button>
        </div>
      </div>
    );
  }

  const sectionTitles = blog.sections.map((s) => s.subtitle);

  // 1) Share handler (Web Share API or copy fallback)
  const shareArticle = () => {
    const shareData = {
      title: blog.title,
      text: blog.excerpt,
      url: window.location.href,
    };

    if (navigator.share) {
      navigator.share(shareData).catch((err) => {
        console.error("Share failed:", err);
      });
    } else {
      navigator.clipboard
        .writeText(shareData.url)
        .then(() => setShareSnackbarOpen(true))
        .catch((err) => console.error("Copy failed:", err));
    }
  };

  // 2) Bookmark handler
  const bookmarkArticle = () => {
    const title = blog.title;
    const url = window.location.href;
    const win: any = window;

    let succeeded = false;
    try {
      // Firefox <=22
      if (win.sidebar && win.sidebar.addPanel) {
        win.sidebar.addPanel(title, url, "");
        succeeded = true;
      }
      // IE Favorites
      else if (win.external && win.external.AddFavorite) {
        win.external.AddFavorite(url, title);
        succeeded = true;
      }
    } catch (e) {
      console.warn("Programmatic bookmark failed", e);
    }

    if (!succeeded) {
      setBookmarkSnackbarOpen(true);
    }
  };

  return (
    <>
      <div className="min-h-screen bg-gradient-to-br from-gray-50 to-white">
        <Container maxWidth="lg" className="py-8">
          <div className="flex flex-col lg:flex-row gap-8">
            {/* Sidebar */}
            <div className="lg:w-1/4 order-2 lg:order-1">
              <div className="sticky top-8 z-10">
                <BlogSidebar sections={sectionTitles} />
              </div>
            </div>

            {/* Main Content */}
            <div className="lg:w-3/4 order-1 lg:order-2">
              {/* Back Button */}
              <Button
                onClick={() => navigate(-1)}
                className="mb-6 text-gray-600 hover:text-gray-800 normal-case"
                startIcon={<ArrowLeft className="w-4 h-4" />}
              >
                Back to Blog
              </Button>

              {/* Category Badge */}
              <div className="mb-4">
                <span className="bg-blue-100 text-blue-800 px-3 py-1 rounded-full text-sm font-medium">
                  {blog.category}
                </span>
              </div>

              {/* Title */}
              <Typography
                variant="h3"
                className="font-bold mb-6 text-gray-900 leading-tight"
              >
                {blog.title}
              </Typography>

              {/* Author & Meta Info */}
              <div className="flex items-center gap-4 mb-6 p-4 bg-white rounded-lg border border-gray-200">
                <Avatar className="w-12 h-12 bg-blue-600">
                  {blog.author[0]}
                </Avatar>
                <div className="flex-1">
                  <Typography variant="body1" className="font-semibold text-gray-900">
                    {blog.author}
                  </Typography>
                  <div className="flex items-center gap-4 text-sm text-gray-600 mt-1">
                    <div className="flex items-center gap-1">
                      <Calendar className="w-4 h-4" />
                      <span>{new Date(blog.date).toLocaleDateString()}</span>
                    </div>
                    <div className="flex items-center gap-1">
                      <Clock className="w-4 h-4" />
                      <span>{blog.readTime}</span>
                    </div>
                  </div>
                </div>
                
                {/* Share & Bookmark */}
                <div className="flex items-center gap-2">
                  <Button
                    size="small"
                    onClick={shareArticle}
                    className="min-w-0 p-2"
                  >
                    <Share2 className="w-4 h-4" />
                  </Button>
                  <Button
                    size="small"
                    onClick={bookmarkArticle}
                    className="min-w-0 p-2"
                  >
                    <Bookmark className="w-4 h-4" />
                  </Button>
                </div>
              </div>

              {/* Featured Image/Video */}
              {blog.image && (
                /\.(mp4|webm)$/i.test(blog.image) ? (
                  <video
                    src={blog.image}
                    autoPlay
                    loop
                    muted
                    controls
                    className="w-full rounded-2xl mb-8 shadow-lg"
                    style={{ maxHeight: "400px", objectFit: "cover" }}
                  />
                ) : (
                  <img
                    src={blog.image}
                    alt={blog.title}
                    className="w-full rounded-2xl mb-8 shadow-lg"
                    style={{ maxHeight: "400px", objectFit: "cover" }}
                  />
                )
              )}

              {/* Introduction */}
              <div className="bg-blue-50 border-l-4 border-blue-500 p-6 rounded-r-lg mb-8">
                <Typography
                  variant="subtitle1"
                  className="text-gray-700 leading-relaxed font-medium"
                >
                  {blog.introText}
                </Typography>
              </div>

              {/* Tags */}
              <div className="flex flex-wrap gap-2 mb-8">
                {blog.tags.map((tag) => (
                  <Chip
                    key={tag}
                    label={`#${tag}`}
                    variant="outlined"
                    size="small"
                    className="border-gray-300 text-gray-600 hover:bg-gray-100"
                  />
                ))}
              </div>

              {/* Article Sections */}
              <article className="prose prose-lg max-w-none">
                {blog.sections.map((section, idx) => {
                  const secId = section.subtitle
                    .replace(/\s+/g, "-")
                    .toLowerCase();
                  return (
                    <section
                      key={idx}
                      id={secId}
                      className="mb-12 scroll-mt-24"
                    >
                      <Typography
                        variant="h4"
                        className="font-bold mb-6 text-gray-900 border-b border-gray-200 pb-2"
                      >
                        {section.subtitle}
                      </Typography>

                      {Array.isArray(section.content) ? (
                        <div className="space-y-6">
                          {section.content.map((item, i) => (
                            <div
                              key={i}
                              className="bg-white p-6 rounded-lg border border-gray-200 shadow-sm"
                            >
                              <Typography
                                variant="h6"
                                className="font-bold text-gray-900 mb-3"
                              >
                                {item.label}
                              </Typography>
                              <Typography className="text-gray-700 leading-relaxed">
                                {item.text}
                              </Typography>
                            </div>
                          ))}
                        </div>
                      ) : (
                        <Typography className="text-gray-700 leading-relaxed mb-6 whitespace-pre-line">
                          {section.content}
                        </Typography>
                      )}

                      {section.video && (
                        <video
                          src={section.video}
                          controls
                          className="w-full rounded-xl mt-6 mb-6 shadow-lg"
                          style={{ maxHeight: "400px" }}
                        />
                      )}
                      {section.image && (
                        <img
                          src={section.image}
                          alt={section.subtitle}
                          className="w-full rounded-xl mt-6 mb-6 shadow-lg"
                          style={{ maxHeight: "400px", objectFit: "cover" }}
                        />
                      )}
                    </section>
                  );
                })}
              </article>

              {/* Article Footer */}
              <Divider className="my-8" />
              <Box className="bg-gradient-to-r from-gray-50 to-blue-50 rounded-xl p-6 mb-8">
                <div className="flex justify-between items-center">
                  <Box>
                    <Typography variant="body2" className="text-gray-600 mb-1">
                      Written with ❤️ by
                    </Typography>
                    <Typography variant="h6" className="font-semibold text-gray-900">
                      {blog.author}
                    </Typography>
                  </Box>
                  <Button
                    variant="contained"
                    size="small"
                    onClick={shareArticle}
                    className="bg-blue-600 hover:bg-blue-700 normal-case"
                  >
                    Share Article
                  </Button>
                </div>
              </Box>

              {/* Related Posts / Navigation */}
              <div className="flex justify-between items-center pt-6 border-t border-gray-200">
                <Link
                  to="/blog"
                  className="text-blue-600 hover:text-blue-800 font-medium flex items-center gap-2"
                >
                  <ArrowLeft className="w-4 h-4" />
                  Back to all posts
                </Link>
                <Button
                  variant="outlined"
                  size="small"
                  className="border-gray-300 text-gray-700 hover:bg-gray-100 normal-case"
                >
                  Save for Later
                </Button>
              </div>
            </div>
          </div>
        </Container>
      </div>

      {/* Share fallback Snackbar */}
      <Snackbar
        open={shareSnackbarOpen}
        autoHideDuration={3000}
        onClose={() => setShareSnackbarOpen(false)}
        anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
      >
        <Alert severity="success" onClose={() => setShareSnackbarOpen(false)}>
          Link copied to clipboard!
        </Alert>
      </Snackbar>

      {/* Bookmark fallback Snackbar */}
      <Snackbar
        open={bookmarkSnackbarOpen}
        autoHideDuration={5000}
        onClose={() => setBookmarkSnackbarOpen(false)}
        anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
      >
        <Alert severity="info" onClose={() => setBookmarkSnackbarOpen(false)}>
          Press Ctrl +D (Windows) or Cmd +D (Mac) to bookmark this page.
        </Alert>
      </Snackbar>
      <Footer setIsOpen={setIsOpen}/>
    </>
  );
}
