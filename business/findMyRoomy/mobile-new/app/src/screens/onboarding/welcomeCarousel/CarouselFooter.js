import React from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  Dimensions,
} from 'react-native';

const { width } = Dimensions.get('window');

export const CarouselFooter = ({ 
  index = 0, 
  totalSlides = 4, 
  onNext, 
  onSkip,
  handleNext,
  skipText = 'Skip',
  nextText = 'Next',
  showSkip = true,
  showNext = true,
  theme = 'light' // 'light' or 'dark'
}) => {
  const isLastSlide = index === totalSlides - 1;
  const styles = getStyles(theme);
  
  const renderDots = () => {
    return Array.from({ length: totalSlides }, (_, i) => (
      <View
        key={i}
        style={[
          styles.dot,
          i === index && styles.activeDot,
        ]}
      />
    ));
  };

  return (
    <View style={styles.footer}>
      {/* Skip Button */}
      <TouchableOpacity
        style={styles.skipButton}
        onPress={onSkip}
        activeOpacity={0.7}
        accessible={true}
        accessibilityLabel={skipText}
        accessibilityRole="button"
      >
        {showSkip && <Text style={styles.skipText}>{skipText}</Text>}
      </TouchableOpacity>

      {/* Dots Indicator */}
      <View style={styles.dotsContainer}>
        {renderDots()}
      </View>

      {/* Next Button */}
      <TouchableOpacity
        style={styles.nextButton}
        onPress={handleNext}
        activeOpacity={0.8}
        accessible={true}
        accessibilityLabel={isLastSlide ? 'Get Started' : nextText}
        accessibilityRole="button"
      >
        {showNext && (
          <Text style={styles.nextText}>
            {isLastSlide ? 'Get Started' : nextText}
          </Text>
        )}
      </TouchableOpacity>
    </View>
  );
};

// Theme-aware styles
const getStyles = (theme) => {
  const isDark = theme === 'dark';
  
  return StyleSheet.create({
    footer: {
      width: '100%',
      flexDirection: 'row',
      justifyContent: 'space-between',
      alignItems: 'center',
      paddingHorizontal: 24,
      paddingVertical: 16,
      marginBottom: 20,
    },
    skipButton: {
      minWidth: 60,
      paddingVertical: 8,
      paddingHorizontal: 12,
      borderRadius: 20,
      backgroundColor: isDark ? 'rgba(255, 255, 255, 0.1)' : 'rgba(0, 0, 0, 0.05)',
      alignItems: 'center',
      justifyContent: 'center',
    },
    skipText: {
      color: isDark ? '#BFDBFE' : '#6B7280',
      fontSize: 14,
      fontWeight: '500',
    },
    dotsContainer: {
      flexDirection: 'row',
      gap: 8,
      alignItems: 'center',
      justifyContent: 'center',
      flex: 1,
    },
    dot: {
      width: 8,
      height: 8,
      borderRadius: 4,
      backgroundColor: isDark ? '#60A5FA' : '#D1D5DB',
      opacity: 0.5,
      transition: 'all 0.3s ease',
    },
    activeDot: {
      width: 24,
      backgroundColor: isDark ? '#FFFFFF' : '#3B82F6',
      opacity: 1,
    },
    nextButton: {
      minWidth: 60,
      paddingVertical: 8,
      paddingHorizontal: 16,
      borderRadius: 20,
      backgroundColor: isDark ? '#FFFFFF' : '#3B82F6',
      alignItems: 'center',
      justifyContent: 'center',
      shadowColor: '#000',
      shadowOffset: {
        width: 0,
        height: 2,
      },
      shadowOpacity: 0.1,
      shadowRadius: 4,
      elevation: 3,
    },
    nextText: {
      color: isDark ? '#1E40AF' : '#FFFFFF',
      fontSize: 14,
      fontWeight: '600',
    },
  });
};

export default CarouselFooter;