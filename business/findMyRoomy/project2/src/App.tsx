import { Routes, Route } from 'react-router-dom';
import LandingPage from './pages/LandingPage'; // Your main page
import BlogList from './pages/Blogs/BlogList';
import SingleBlog from './pages/Blogs/SingleBlog';
import { useState } from 'react';
import ContactUs from './components/ContactUs';

function App() {
    const [isOpen, setIsOpen] = useState(false);
  return (
    <Routes>
        
      <Route path="/" element={<LandingPage setIsOpen={setIsOpen} isOpen = {isOpen}/>} />
      <Route path="/blog" element={<BlogList setIsOpen={setIsOpen} isOpen={isOpen}/>} />
      <Route path="/blog/:id" element={<SingleBlog setIsOpen={setIsOpen} isOpen={isOpen}/>} />
    </Routes>
  );
}

export default App;
