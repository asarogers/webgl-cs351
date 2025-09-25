import { Feather, FontAwesome } from '@expo/vector-icons';
import React, { useEffect, useRef, useState } from 'react';
import {
  ActivityIndicator,
  Animated,
  Dimensions,
  KeyboardAvoidingView,
  Platform,
  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View
} from 'react-native';

import { SafeAreaView } from 'react-native-safe-area-context';
import { navigateTo } from "../../../../components/onboardingStepsHelper";
import authService from '../../../../database/authService';
import ForgotPasswordModal from './ForgotPasswordModal'; // Import the modal

const { width, height } = Dimensions.get('window');

// Animated particle component
const AnimatedParticle = ({ delay = 0 }) => {
  const animatedValue = useRef(new Animated.Value(0)).current;
  const opacityValue = useRef(new Animated.Value(0)).current;
  
  useEffect(() => {
    const startAnimation = () => {
      Animated.parallel([
        Animated.timing(animatedValue, {
          toValue: 1,
          duration: 8000 + Math.random() * 4000,
          useNativeDriver: true,
        }),
        Animated.sequence([
          Animated.timing(opacityValue, {
            toValue: 0.3 + Math.random() * 0.4,
            duration: 1000,
            useNativeDriver: true,
          }),
          Animated.timing(opacityValue, {
            toValue: 0,
            duration: 1000,
            useNativeDriver: true,
          }),
        ]),
      ]).start(() => {
        animatedValue.setValue(0);
        opacityValue.setValue(0);
        startAnimation();
      });
    };

    const timer = setTimeout(startAnimation, delay);
    return () => clearTimeout(timer);
  }, []);

  const translateY = animatedValue.interpolate({
    inputRange: [0, 1],
    outputRange: [height + 50, -50],
  });

  const translateX = animatedValue.interpolate({
    inputRange: [0, 0.5, 1],
    outputRange: [0, 20 + Math.random() * 40, 0],
  });

  return (
    <Animated.View
      style={[
        styles.particle,
        {
          left: Math.random() * width,
          transform: [{ translateY }, { translateX }],
          opacity: opacityValue,
        },
      ]}
    />
  );
};

// Gradient background component
const GradientBackground = () => {
  const animatedValue = useRef(new Animated.Value(0)).current;

  useEffect(() => {
    const animate = () => {
      Animated.sequence([
        Animated.timing(animatedValue, {
          toValue: 1,
          duration: 3000,
          useNativeDriver: false,
        }),
        Animated.timing(animatedValue, {
          toValue: 0,
          duration: 3000,
          useNativeDriver: false,
        }),
      ]).start(() => animate());
    };
    animate();
  }, []);

  const backgroundColor = animatedValue.interpolate({
    inputRange: [0, 1],
    outputRange: ['#111827', '#1F2937'],
  });

  return (
    <Animated.View style={[styles.gradientBackground, { backgroundColor }]}>
      {/* Create multiple particles */}
      {Array.from({ length: 15 }).map((_, index) => (
        <AnimatedParticle key={index} delay={index * 400} />
      ))}
    </Animated.View>
  );
};

const LoginScreen = ({ navigation }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [showForgotPasswordModal, setShowForgotPasswordModal] = useState(false); // Add modal state
  
  // Animation values
  const fadeAnim = useRef(new Animated.Value(0)).current;
  const slideAnim = useRef(new Animated.Value(50)).current;
  const scaleAnim = useRef(new Animated.Value(0.9)).current;

  useEffect(() => {
    // Entrance animation
    Animated.parallel([
      Animated.timing(fadeAnim, {
        toValue: 1,
        duration: 1000,
        useNativeDriver: true,
      }),
      Animated.timing(slideAnim, {
        toValue: 0,
        duration: 800,
        useNativeDriver: true,
      }),
      Animated.spring(scaleAnim, {
        toValue: 1,
        tension: 100,
        friction: 8,
        useNativeDriver: true,
      }),
    ]).start();
  }, []);

  const loginWithGoogle = async () => {
    setError('');
    setLoading(true);
    try {
      const result = await authService.signInWithGoogle();
      if (result.success) {
        // Navigate to main app - the auth state change will be handled by HomeScreen
        navigation.reset({
          index: 0,
          routes: [{ name: 'Main' }]
        });
      } else {
        setError(result.error || 'Google login failed. Please try again.');
      }
    } catch (e) {
      setError('Something went wrong. Please try again.');
    } finally {
      setLoading(false);
    }
  };
  

  const loginWithEmail = async () => {
    // Basic validation
    if (!email.trim()) {
      setError('Please enter your email address');
      return;
    }
    
    if (!password.trim()) {
      setError('Please enter your password');
      return;
    }

    // Basic email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      setError('Please enter a valid email address');
      return;
    }

    setError('');
  setLoading(true);
  
  try {
    const result = await authService.signInWithEmail(email, password);

    if (result.success) {
      // Navigate to main app
      navigation.reset({
        index: 0,
        routes: [{ name: 'Main' }]
      });
    } else {
      setError(result.error || 'Login failed. Please try again.');
    }
  } catch (error) {
    console.error('Login error:', error);
    setError('Something went wrong. Please try again.');
  } finally {
    setLoading(false);
  }
  };

  const handleButtonPress = (callback) => {
    Animated.sequence([
      Animated.timing(scaleAnim, {
        toValue: 0.95,
        duration: 100,
        useNativeDriver: true,
      }),
      Animated.timing(scaleAnim, {
        toValue: 1,
        duration: 100,
        useNativeDriver: true,
      }),
    ]).start();
    callback();
  };

  // Handler for forgot password
  const handleForgotPassword = () => {
    setShowForgotPasswordModal(true);
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <GradientBackground />
      
      <KeyboardAvoidingView
        style={styles.container}
        behavior={Platform.OS === 'ios' ? 'padding' : undefined}
      >
        <Animated.View
          style={[
            styles.content,
            {
              opacity: fadeAnim,
              transform: [
                { translateY: slideAnim },
                { scale: scaleAnim },
              ],
            },
          ]}
        >
          <Text style={styles.logo}>MyRoomie</Text>

          <View style={styles.header}>
            <Text style={styles.title}>Welcome back</Text>
            <Text style={styles.subtitle}>Sign in to find your roommate</Text>
          </View>

          {/* ERROR MESSAGE DISPLAY */}
          {error ? (
            <View style={styles.errorContainer}>
              <Text style={styles.errorText}>{error}</Text>
            </View>
          ) : null}

          {/* Email Input */}
          <View style={[styles.inputWrapper, error && (error.includes('email') || error.includes('Email')) && styles.inputError]}>
            <Feather name="mail" size={18} color="#9CA3AF" style={styles.inputIcon} />
            <TextInput
              placeholder="Email address"
              placeholderTextColor="#9CA3AF"
              style={styles.input}
              keyboardType="email-address"
              autoCapitalize="none"
              value={email}
              onChangeText={(text) => {
                setEmail(text);
                if (error) setError(''); // Clear error when user starts typing
              }}
              editable={!loading}
            />
          </View>

          {/* Password Input */}
          <View style={[styles.inputWrapper, error && error.includes('password') && styles.inputError]}>
            <Feather name="lock" size={18} color="#9CA3AF" style={styles.inputIcon} />
            <TextInput
              placeholder="Password"
              placeholderTextColor="#9CA3AF"
              style={styles.input}
              secureTextEntry={!showPassword}
              value={password}
              onChangeText={(text) => {
                setPassword(text);
                if (error) setError(''); // Clear error when user starts typing
              }}
              editable={!loading}
            />
            <TouchableOpacity onPress={() => setShowPassword(!showPassword)} disabled={loading}>
              <Feather 
                name={showPassword ? "eye-off" : "eye"} 
                size={18} 
                color="#9CA3AF" 
                style={styles.eyeIcon} 
              />
            </TouchableOpacity>
          </View>

          {/* Updated forgot password button */}
          <TouchableOpacity 
            style={styles.forgotLink} 
            onPress={handleForgotPassword}
            disabled={loading}
          >
            <Text style={[styles.forgotText, loading && styles.disabledText]}>
              Forgot password?
            </Text>
          </TouchableOpacity>

          {/* LOGIN BUTTON WITH LOADING STATE */}
          <TouchableOpacity 
            style={[styles.loginButton, loading && styles.loginButtonDisabled]} 
            onPress={() => handleButtonPress(loginWithEmail)}
            disabled={loading}
          >
            {loading ? (
              <ActivityIndicator color="#FFFFFF" size="small" />
            ) : (
              <Text style={styles.loginText}>Sign in</Text>
            )}
          </TouchableOpacity>

          <View style={styles.dividerRow}>
            <View style={styles.divider} />
            <Text style={styles.dividerText}>or continue with</Text>
            <View style={styles.divider} />
          </View>

          {/* Google */}
          <TouchableOpacity 
            style={[styles.oauthButton, loading && styles.oauthButtonDisabled]}
            onPress={() => handleButtonPress(() => loginWithGoogle())}
            disabled={loading}
          >
            <FontAwesome name="google" size={18} color="#FFFFFF" style={styles.oauthIcon} />
            <Text style={styles.oauthText}>Continue with Google</Text>
          </TouchableOpacity>

          <TouchableOpacity 
            onPress={() => navigation.navigate('SignUp')} 
            disabled={loading}
          >
            <Text style={[styles.signupLink, loading && styles.disabledText]}>
              Don't have an account? <Text style={styles.signupHighlight}>Sign up</Text>
            </Text>
          </TouchableOpacity>
        </Animated.View>
      </KeyboardAvoidingView>

      {/* Forgot Password Modal */}
      <ForgotPasswordModal
        visible={showForgotPasswordModal}
        onClose={() => setShowForgotPasswordModal(false)}
      />
    </SafeAreaView>
  );
};

export default LoginScreen;

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#111827',
  },
  gradientBackground: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
  },
  particle: {
    position: 'absolute',
    width: 4,
    height: 4,
    borderRadius: 2,
    backgroundColor: '#3B82F6',
    shadowColor: '#3B82F6',
    shadowOffset: { width: 0, height: 0 },
    shadowOpacity: 0.8,
    shadowRadius: 4,
    elevation: 5,
  },
  container: {
    flex: 1,
    padding: 24,
    justifyContent: 'center',
  },
  content: {
    flex: 1,
    justifyContent: 'center',
  },
  logo: {
    color: '#FFFFFF',
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 12,
  },
  header: {
    marginBottom: 24,
    alignItems: 'center',
  },
  title: {
    fontSize: 22,
    fontWeight: '700',
    color: '#FFFFFF',
  },
  subtitle: {
    fontSize: 14,
    color: '#9CA3AF',
    marginTop: 4,
  },
  // ERROR STYLES
  errorContainer: {
    backgroundColor: 'rgba(239, 68, 68, 0.1)',
    borderWidth: 1,
    borderColor: 'rgba(239, 68, 68, 0.3)',
    borderRadius: 8,
    padding: 12,
    marginBottom: 16,
  },
  errorText: {
    color: '#FCA5A5',
    fontSize: 14,
    textAlign: 'center',
  },
  inputWrapper: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: 'rgba(31, 41, 55, 0.8)',
    borderRadius: 8,
    paddingHorizontal: 12,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: 'rgba(59, 130, 246, 0.1)',
  },
  inputError: {
    borderColor: 'rgba(239, 68, 68, 0.5)',
  },
  inputIcon: {
    marginRight: 8,
  },
  input: {
    flex: 1,
    paddingVertical: 12,
    color: '#FFFFFF',
    fontSize: 15,
  },
  eyeIcon: {
    marginLeft: 8,
  },
  forgotLink: {
    alignItems: 'flex-end',
    marginBottom: 20,
  },
  forgotText: {
    color: '#60A5FA',
    fontSize: 13,
  },
  loginButton: {
    backgroundColor: '#3B82F6',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
    marginBottom: 24,
    shadowColor: '#3B82F6',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 8,
    elevation: 8,
  },
  loginButtonDisabled: {
    backgroundColor: '#374151',
    shadowOpacity: 0,
    elevation: 0,
  },
  loginText: {
    color: '#FFFFFF',
    fontSize: 15,
    fontWeight: '600',
  },
  dividerRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 16,
  },
  divider: {
    flex: 1,
    height: 1,
    backgroundColor: '#374151',
  },
  dividerText: {
    marginHorizontal: 12,
    color: '#9CA3AF',
    fontSize: 12,
  },
  oauthButton: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: 'rgba(55, 65, 81, 0.8)',
    paddingVertical: 12,
    borderRadius: 8,
    marginBottom: 12,
    paddingHorizontal: 16,
    borderWidth: 1,
    borderColor: 'rgba(59, 130, 246, 0.1)',
  },
  oauthButtonDisabled: {
    backgroundColor: 'rgba(55, 65, 81, 0.4)',
  },
  oauthIcon: {
    marginRight: 12,
  },
  oauthText: {
    color: '#FFFFFF',
    fontSize: 15,
    fontWeight: '500',
  },
  signupLink: {
    color: '#9CA3AF',
    textAlign: 'center',
    marginTop: 24,
    fontSize: 14,
  },
  signupHighlight: {
    color: '#60A5FA',
    fontWeight: '600',
  },
  disabledText: {
    opacity: 0.5,
  },
});