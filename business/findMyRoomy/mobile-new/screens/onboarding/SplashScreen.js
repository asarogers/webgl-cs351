import React, { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, Animated, Image } from 'react-native';
import { FontAwesome5, MaterialIcons, Entypo } from '@expo/vector-icons';

const SplashScreen = ({ navigation, isFirstTime }) => {
  const fadeAnim = useRef(new Animated.Value(0)).current;
  const progressAnim = useRef(new Animated.Value(0)).current;
  const scaleAnim = useRef(new Animated.Value(0.8)).current;
  const pinAnimations = useRef([
    new Animated.Value(0),
    new Animated.Value(0),
    new Animated.Value(0),
    new Animated.Value(0),
  ]).current;
  const pinBounceAnimations = useRef([
    new Animated.Value(0),
    new Animated.Value(0),
    new Animated.Value(0),
    new Animated.Value(0),
  ]).current;
  const titleShineAnim = useRef(new Animated.Value(0)).current;
  const featureRotateAnim = useRef(new Animated.Value(0)).current;

  const [currentMessageIndex, setCurrentMessageIndex] = useState(0);

  const loadingMessages = [
    "Setting up your vibe...",
    "Matching your preferences...",
    "Finding real people, not randos...",
    "Drawing your custom zone...",
    "Checking for safe & verified listings...",
    "You're almost there..."
  ];

  useEffect(() => {
    // Set up message cycling with proper cleanup
    const messageInterval = setInterval(() => {
      setCurrentMessageIndex(prev => {
        if (prev < loadingMessages.length - 1) {
          return prev + 1;
        }
        return prev; // Stop at last message
      });
    }, 700);

    // Start animations with proper timing
    const startAnimations = () => {
      // Stagger animations for a more polished feel
      const animations = [
        // Scale up the main content
        Animated.timing(scaleAnim, {
          toValue: 1,
          duration: 800,
          useNativeDriver: true,
        }),
        // Fade in content
        Animated.timing(fadeAnim, {
          toValue: 1,
          duration: 1000,
          useNativeDriver: true,
        }),
        // Animate pins with staggered timing
        Animated.stagger(300, pinAnimations.map(anim => 
          Animated.timing(anim, {
            toValue: 1,
            duration: 600,
            useNativeDriver: true,
          })
        )),
      ];

      Animated.parallel(animations).start();

      // Continuous pin bouncing animation
      const bounceAnimations = pinBounceAnimations.map((anim, index) => 
        Animated.loop(
          Animated.sequence([
            Animated.delay(index * 400), // Stagger the bouncing
            Animated.timing(anim, {
              toValue: 1,
              duration: 800,
              useNativeDriver: true,
            }),
            Animated.timing(anim, {
              toValue: 0,
              duration: 800,
              useNativeDriver: true,
            }),
          ])
        )
      );

      bounceAnimations.forEach(anim => anim.start());

      // Title shine effect
      const titleShineLoop = Animated.loop(
        Animated.sequence([
          Animated.delay(2000),
          Animated.timing(titleShineAnim, {
            toValue: 1,
            duration: 1500,
            useNativeDriver: true,
          }),
          Animated.timing(titleShineAnim, {
            toValue: 0,
            duration: 1500,
            useNativeDriver: true,
          }),
        ])
      );
      titleShineLoop.start();

      // Feature icons rotation
      const featureRotateLoop = Animated.loop(
        Animated.timing(featureRotateAnim, {
          toValue: 1,
          duration: 8000,
          useNativeDriver: true,
        })
      );
      featureRotateLoop.start();

      // Animate loading bar from 0% to 100% width
      Animated.timing(progressAnim, {
        toValue: 1,
        duration: 2500,
        useNativeDriver: false,
      }).start(() => {
        // Navigate after progress completes
        setTimeout(() => {
          if(isFirstTime){
            navigation.replace('Welcome');
          }else{
            navigation.replace("Auth", { screen: "Login" });
          }
          
        }, 500);
      });
    };

    // Start animations with a small delay to prevent useInsertionEffect warning
    const animationTimer = setTimeout(startAnimations, 100);

    // Cleanup function
    return () => {
      clearInterval(messageInterval);
      clearTimeout(animationTimer);
    };
  }, [navigation, fadeAnim, progressAnim, scaleAnim, pinAnimations, pinBounceAnimations, titleShineAnim, featureRotateAnim, loadingMessages.length]);

  const progressWidth = progressAnim.interpolate({
    inputRange: [0, 1],
    outputRange: ['0%', '100%'],
  });

  const pinStyles = pinAnimations.map((anim, index) => ({
    opacity: anim,
    transform: [
      {
        scale: anim.interpolate({
          inputRange: [0, 1],
          outputRange: [0, 1],
        }),
      },
      {
        translateY: pinBounceAnimations[index].interpolate({
          inputRange: [0, 1],
          outputRange: [0, -8],
        }),
      },
    ],
  }));

  const titleShineTransform = titleShineAnim.interpolate({
    inputRange: [0, 1],
    outputRange: [0, 1],
  });

  const featureRotation = featureRotateAnim.interpolate({
    inputRange: [0, 1],
    outputRange: ['0deg', '360deg'],
  });

  return (
    <View style={styles.container}>
      {/* Floating map markers with animation */}
      <Animated.View style={[styles.pinTopLeft, pinStyles[0]]}>
        <FontAwesome5 name="map-marker-alt" size={16} color="#F8FAFC" />
      </Animated.View>
      <Animated.View style={[styles.pinTopRight, pinStyles[1]]}>
        <FontAwesome5 name="map-marker-alt" size={16} color="#F8FAFC" />
      </Animated.View>
      <Animated.View style={[styles.pinBottomLeft, pinStyles[2]]}>
        <FontAwesome5 name="map-marker-alt" size={16} color="#F8FAFC" />
      </Animated.View>
      <Animated.View style={[styles.pinBottomRight, pinStyles[3]]}>
        <FontAwesome5 name="map-marker-alt" size={16} color="#F8FAFC" />
      </Animated.View>

      {/* Main content with scale animation */}
      <Animated.View style={[
        styles.mainContent,
        {
          opacity: fadeAnim,
          transform: [{ scale: scaleAnim }],
        }
      ]}>
        {/* App Icon Container */}
        <View style={styles.iconContainer}>
          <FontAwesome5 name="home" size={32} color="#F8FAFC" />
        </View>

        <Animated.Text style={[
          styles.title,
          {
            opacity: titleShineTransform.interpolate({
              inputRange: [0, 0.5, 1],
              outputRange: [1, 0.7, 1],
            }),
            transform: [
              {
                scale: titleShineTransform.interpolate({
                  inputRange: [0, 0.5, 1],
                  outputRange: [1, 1.05, 1],
                }),
              },
            ],
          }
        ]}>
          MyRoomie
        </Animated.Text>

        <Text style={styles.subtitle}>
          Find your perfect roommate match
        </Text>

        <View style={styles.featuresContainer}>
          <View style={styles.feature}>
            <Animated.View style={[
              styles.featureIconContainer,
              { transform: [{ rotate: featureRotation }] }
            ]}>
              <FontAwesome5 name="map-marked-alt" size={20} color="#60A5FA" />
            </Animated.View>
            <Text style={styles.featureText}>Map Search</Text>
          </View>
          <View style={styles.feature}>
            <Animated.View style={[
              styles.featureIconContainer,
              { transform: [{ rotate: featureRotation }] }
            ]}>
              <MaterialIcons name="people" size={20} color="#10B981" />
            </Animated.View>
            <Text style={styles.featureText}>Smart Match</Text>
          </View>
          <View style={styles.feature}>
            <Animated.View style={[
              styles.featureIconContainer,
              { transform: [{ rotate: featureRotation }] }
            ]}>
              <Entypo name="shield" size={20} color="#22C55E" />
            </Animated.View>
            <Text style={styles.featureText}>Verified</Text>
          </View>
        </View>

        {/* Loading Bar */}
        <View style={styles.progressBarBackground}>
          <Animated.View style={[styles.progressBarFill, { width: progressWidth }]} />
        </View>

        <Text style={styles.loadingText}>
          {loadingMessages[currentMessageIndex]}
        </Text>
      </Animated.View>
    </View>
  );
};

export default SplashScreen;

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1F2937', // Neutral/Navy
    justifyContent: 'center',
    alignItems: 'center',
    paddingHorizontal: 20,
  },
  mainContent: {
    alignItems: 'center',
  },
  iconContainer: {
    backgroundColor: '#3B82F6', // Primary/Blue
    padding: 24,
    borderRadius: 24,
    marginBottom: 24,
    shadowColor: '#3B82F6',
    shadowOffset: {
      width: 0,
      height: 8,
    },
    shadowOpacity: 0.3,
    shadowRadius: 16,
    elevation: 8,
  },
  title: {
    fontSize: 36,
    fontWeight: '800',
    color: '#F8FAFC', // Neutral/Background (used as text on dark)
    marginBottom: 8,
    textAlign: 'center',
  },
  subtitle: {
    fontSize: 18,
    color: '#E5E7EB', // Neutral/Gray Light
    marginBottom: 32,
    textAlign: 'center',
    fontWeight: '400',
  },
  featuresContainer: {
    flexDirection: 'row',
    gap: 32,
    marginBottom: 40,
  },
  feature: {
    alignItems: 'center',
  },
  featureIconContainer: {
    backgroundColor: '#374151', // Neutral/Slate
    padding: 12,
    borderRadius: 12,
    marginBottom: 8,
    borderWidth: 1,
    borderColor: '#6B7280', // Neutral/Gray Medium
  },
  featureText: {
    color: '#E5E7EB', // Neutral/Gray Light
    fontSize: 13,
    fontWeight: '500',
    textAlign: 'center',
  },

  // Progress bar
  progressBarBackground: {
    width: 240,
    height: 6,
    backgroundColor: '#374151', // Neutral/Slate
    borderRadius: 3,
    overflow: 'hidden',
    marginBottom: 20,
  },
  progressBarFill: {
    height: 6,
    backgroundColor: '#3B82F6', // Primary/Blue
    borderRadius: 3,
    shadowColor: '#3B82F6',
    shadowOffset: {
      width: 0,
      height: 2,
    },
    shadowOpacity: 0.5,
    shadowRadius: 4,
    elevation: 2,
  },

  loadingText: {
    color: '#6B7280', // Neutral/Gray Medium
    fontSize: 15,
    fontStyle: 'italic',
    fontWeight: '400',
  },

  // Pin positions with updated colors - closer together and as map markers
  pinTopLeft: {
    position: 'absolute',
    top: 120,
    left: 100,
    width: 24,
    height: 30,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#60A5FA', // Map/Marker Blue
    borderRadius: 12,
    borderBottomLeftRadius: 0,
    borderBottomRightRadius: 0,
    shadowColor: '#60A5FA',
    shadowOffset: {
      width: 0,
      height: 4,
    },
    shadowOpacity: 0.6,
    shadowRadius: 8,
    elevation: 8,
  },
  pinTopRight: {
    position: 'absolute',
    top: 100,
    right: 80,
    width: 24,
    height: 30,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#10B981', // Tag/Green
    borderRadius: 12,
    borderBottomLeftRadius: 0,
    borderBottomRightRadius: 0,
    shadowColor: '#10B981',
    shadowOffset: {
      width: 0,
      height: 4,
    },
    shadowOpacity: 0.6,
    shadowRadius: 8,
    elevation: 8,
  },
  pinBottomLeft: {
    position: 'absolute',
    bottom: 140,
    left: 80,
    width: 24,
    height: 30,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#22C55E', // Status/Online Dot
    borderRadius: 12,
    borderBottomLeftRadius: 0,
    borderBottomRightRadius: 0,
    shadowColor: '#22C55E',
    shadowOffset: {
      width: 0,
      height: 4,
    },
    shadowOpacity: 0.6,
    shadowRadius: 8,
    elevation: 8,
  },
  pinBottomRight: {
    position: 'absolute',
    bottom: 160,
    right: 100,
    width: 24,
    height: 30,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#3B82F6', // Primary/Blue
    borderRadius: 12,
    borderBottomLeftRadius: 0,
    borderBottomRightRadius: 0,
    shadowColor: '#3B82F6',
    shadowOffset: {
      width: 0,
      height: 4,
    },
    shadowOpacity: 0.6,
    shadowRadius: 8,
    elevation: 8,
  },
});