import React from 'react';
import {
  View,
  Text,
  StyleSheet,
  Dimensions,
  StatusBar,
  TouchableOpacity,
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { FontAwesome5 } from '@expo/vector-icons';
import CarouselFooter from './CarouselFooter';

const { width, height } = Dimensions.get('window');

const TrustAndReadyScreen = ({ onNext, onSkip,handleNext, index = 2 }) => {
  const features = [
    {
      icon: 'check-circle',
      text: 'Verified profiles only',
      description: 'ID verification & background checks',
      color: '#10B981',
    },
    {
      icon: 'shield-alt',
      text: 'Safe & secure messaging',
      description: 'End-to-end encrypted conversations',
      color: '#3B82F6',
    },
    {
      icon: 'user-tag',
      text: 'Transparent lifestyle tags',
      description: 'Know compatibility upfront',
      color: '#8B5CF6',
    },
  ];

  const renderFeaturePill = (feature, i) => (
    <TouchableOpacity
      key={i}
      style={[styles.featurePill, { borderLeftColor: feature.color }]}
      activeOpacity={0.8}
      accessible={true}
      accessibilityLabel={`${feature.text}: ${feature.description}`}
      accessibilityRole="button"
    >
      <View style={[styles.iconWrapper, { backgroundColor: `${feature.color}20` }]}>
        <FontAwesome5 
          name={feature.icon} 
          size={18} 
          color={feature.color} 
        />
      </View>
      <View style={styles.textWrapper}>
        <Text style={styles.pillText}>{feature.text}</Text>
        <Text style={styles.pillDescription}>{feature.description}</Text>
      </View>
    </TouchableOpacity>
  );

  return (
    <SafeAreaView style={styles.safeArea}>
      <StatusBar barStyle="light-content" backgroundColor="#1E3A8A" />
      <View style={styles.container}>
        {/* Enhanced Decorative Elements */}
        <View style={styles.circle1} />
        <View style={styles.circle2} />
        <View style={styles.circle3} />
        <View style={styles.circle4} />
        <View style={styles.circle5} />
        
        {/* Floating Elements */}
        <View style={styles.floatingElement1} />
        <View style={styles.floatingElement2} />
        
        <View style={styles.contentContainer}>
          {/* Enhanced Icon Container */}
          <View style={styles.iconContainer}>
            <View style={styles.innerIconContainer}>
              <FontAwesome5 name="rocket" size={40} color="#FACC15" />
            </View>
            <View style={styles.iconGlow} />
          </View>
          
          {/* Enhanced Text */}
          <Text style={styles.title}>Roommate Profiles You Can Trust</Text>
          <Text style={styles.subtitle}>
            Every profile you see has gone through verification steps so you can match with real, safe, and compatible people.
          </Text>
          
          {/* Enhanced Feature Pills */}
          <View style={styles.featureList}>
            {features.map(renderFeaturePill)}
          </View>
          
          {/* Trust Badge */}
          <View style={styles.trustBadge}>
            <FontAwesome5 name="certificate" size={16} color="#FACC15" />
            <Text style={styles.trustText}>99.8% verified profiles</Text>
          </View>
        </View>
        
        {/* Footer Always at Bottom */}
        <CarouselFooter
          index={index}
          totalSlides={4}
          handleNext = {handleNext} 
          onSkip={onSkip}
          theme="dark"
        />
      </View>
    </SafeAreaView>
  );
};

export default TrustAndReadyScreen;

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#1E3A8A',
  },
  container: {
    flex: 1,
    width,
    backgroundColor: '#1E3A8A',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: 20,
    position: 'relative',
  },
  contentContainer: {
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
    paddingTop: 40,
  },
  iconContainer: {
    position: 'relative',
    marginBottom: 32,
  },
  innerIconContainer: {
    backgroundColor: '#1E40AF',
    padding: 24,
    borderRadius: 50,
    shadowColor: '#FACC15',
    shadowOffset: {
      width: 0,
      height: 4,
    },
    shadowOpacity: 0.3,
    shadowRadius: 12,
    elevation: 8,
  },
  iconGlow: {
    position: 'absolute',
    top: -5,
    left: -5,
    right: -5,
    bottom: -5,
    borderRadius: 60,
    backgroundColor: '#FACC15',
    opacity: 0.1,
  },
  title: {
    fontSize: 26,
    fontWeight: '700',
    color: '#FFFFFF',
    textAlign: 'center',
    marginBottom: 16,
    paddingHorizontal: 10,
    lineHeight: 32,
  },
  subtitle: {
    fontSize: 16,
    color: '#D1E6FF',
    textAlign: 'center',
    lineHeight: 24,
    marginBottom: 40,
    paddingHorizontal: 10,
    opacity: 0.9,
  },
  featureList: {
    gap: 16,
    width: 400,
    marginBottom: 32,
  },
  featurePill: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#1E40AF',
    paddingVertical: 16,
    paddingHorizontal: 20,
    borderRadius: 16,
    borderLeftWidth: 4,
    shadowColor: '#000',
    shadowOffset: {
      width: 0,
      height: 2,
    },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  iconWrapper: {
    padding: 10,
    borderRadius: 12,
    marginRight: 16,
  },
  textWrapper: {
    flex: 1,
  },
  pillText: {
    fontSize: 16,
    color: '#FFFFFF',
    fontWeight: '600',
    marginBottom: 4,
  },
  pillDescription: {
    fontSize: 13,
    color: '#94A3B8',
    fontWeight: '400',
  },
  trustBadge: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#1E40AF',
    paddingVertical: 8,
    paddingHorizontal: 16,
    borderRadius: 20,
    gap: 8,
    marginBottom: 20,
  },
  trustText: {
    fontSize: 13,
    color: '#FACC15',
    fontWeight: '600',
  },
  // Enhanced decorative elements
  circle1: {
    position: 'absolute',
    top: height * 0.08,
    left: width * 0.15,
    width: 32,
    height: 32,
    borderRadius: 16,
    backgroundColor: '#60A5FA',
    opacity: 0.25,
  },
  circle2: {
    position: 'absolute',
    bottom: height * 0.28,
    right: width * 0.08,
    width: 28,
    height: 28,
    borderRadius: 14,
    backgroundColor: '#10B981',
    opacity: 0.2,
  },
  circle3: {
    position: 'absolute',
    top: height * 0.32,
    right: width * 0.12,
    width: 24,
    height: 24,
    borderRadius: 12,
    backgroundColor: '#8B5CF6',
    opacity: 0.3,
  },
  circle4: {
    position: 'absolute',
    top: height * 0.18,
    right: width * 0.25,
    width: 16,
    height: 16,
    borderRadius: 8,
    backgroundColor: '#FACC15',
    opacity: 0.4,
  },
  circle5: {
    position: 'absolute',
    bottom: height * 0.35,
    left: width * 0.08,
    width: 20,
    height: 20,
    borderRadius: 10,
    backgroundColor: '#93C5FD',
    opacity: 0.3,
  },
  floatingElement1: {
    position: 'absolute',
    top: height * 0.25,
    left: width * 0.05,
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: '#60A5FA',
    opacity: 0.6,
  },
  floatingElement2: {
    position: 'absolute',
    bottom: height * 0.4,
    right: width * 0.2,
    width: 14,
    height: 14,
    borderRadius: 7,
    backgroundColor: '#10B981',
    opacity: 0.5,
  },
});