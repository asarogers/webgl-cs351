import React from 'react';
import {
  View,
  Text,
  StyleSheet,
  TouchableOpacity,
  Dimensions,
  StatusBar,
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { FontAwesome5, MaterialIcons, Entypo } from '@expo/vector-icons';
import CarouselFooter from './CarouselFooter';

const { width, height } = Dimensions.get('window');

const MatchVibe = ({ onNext, onSkip,handleNext, index = 1 }) => {
  const features = [
    {
      icon: 'hand-holding-heart',
      iconSet: FontAwesome5,
      text: 'Lifestyle compatibility',
      description: 'Matches based on daily routines & habits',
    },
    {
      icon: 'label',
      iconSet: MaterialIcons,
      text: 'Clear preference tags',
      description: 'Know what matters most to your match',
    },
    {
      icon: 'message',
      iconSet: Entypo,
      text: 'Instant messaging',
      description: 'Start chatting before you move in',
    },
  ];

  const renderFeaturePill = (feature, index) => {
    const IconComponent = feature.iconSet;
    return (
      <TouchableOpacity
        key={index}
        style={styles.featurePill}
        activeOpacity={0.8}
        accessible={true}
        accessibilityLabel={`Feature: ${feature.text}. ${feature.description}`}
        accessibilityRole="button"
      >
        <View style={styles.pillIconContainer}>
          <IconComponent name={feature.icon} size={16} color="#059669" />
        </View>
        <View style={styles.pillTextContainer}>
          <Text style={styles.pillText}>{feature.text}</Text>
          <Text style={styles.pillDescription}>{feature.description}</Text>
        </View>
      </TouchableOpacity>
    );
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <StatusBar barStyle="light-content" backgroundColor="#22C55E" />
      <View style={styles.container}>
        {/* Decorative bubbles */}
        <View style={styles.decorativeCircle1} />
        <View style={styles.decorativeCircle2} />
        <View style={styles.decorativeCircle3} />

        <View style={styles.contentContainer}>
          <View style={styles.iconContainer}>
            <Entypo name="star" size={36} color="#22C55E" />
          </View>

          <Text style={styles.title}>Match Your Vibe</Text>
          <Text style={styles.subtitle}>
            Our smart algorithm finds compatible roommates based on lifestyle and preferences.
          </Text>

          <View style={styles.featureButtons}>
            {features.map((feature, index) => renderFeaturePill(feature, index))}
          </View>
        </View>

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

export default MatchVibe;

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#22C55E',
  },
  container: {
    flex: 1,
    width,
    backgroundColor: '#22C55E',
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
  decorativeCircle1: {
    position: 'absolute',
    top: height * 0.08,
    left: width * 0.1,
    width: 28,
    height: 28,
    borderRadius: 14,
    backgroundColor: '#BBF7D0',
    opacity: 0.25,
  },
  decorativeCircle2: {
    position: 'absolute',
    bottom: height * 0.25,
    right: width * 0.08,
    width: 36,
    height: 36,
    borderRadius: 18,
    backgroundColor: '#BBF7D0',
    opacity: 0.2,
  },
  decorativeCircle3: {
    position: 'absolute',
    top: height * 0.3,
    right: width * 0.15,
    width: 20,
    height: 20,
    borderRadius: 10,
    backgroundColor: '#A7F3D0',
    opacity: 0.4,
  },
  iconContainer: {
    backgroundColor: '#ECFDF5',
    padding: 20,
    borderRadius: 28,
    marginBottom: 28,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  title: {
    fontSize: 24,
    fontWeight: '700',
    color: '#FFFFFF',
    textAlign: 'center',
    marginBottom: 16,
    paddingHorizontal: 10,
    lineHeight: 30,
  },
  subtitle: {
    fontSize: 16,
    color: '#F0FDF4',
    textAlign: 'center',
    marginBottom: 40,
    lineHeight: 24,
    paddingHorizontal: 10,
    opacity: 0.9,
  },
  featureButtons: {
    width: '100%',
    gap: 16,
    marginBottom: 40,
  },
  featurePill: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
    paddingVertical: 16,
    paddingHorizontal: 20,
    borderRadius: 24,
    backgroundColor: '#FFFFFF',
    alignSelf: 'center',
    minWidth: width * 0.85,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 2,
  },
  pillIconContainer: {
    backgroundColor: '#DCFCE7',
    padding: 8,
    borderRadius: 12,
  },
  pillTextContainer: {
    flex: 1,
  },
  pillText: {
    fontSize: 15,
    color: '#065F46',
    fontWeight: '600',
    marginBottom: 2,
  },
  pillDescription: {
    fontSize: 13,
    color: '#64748B',
    fontWeight: '400',
  },
});
