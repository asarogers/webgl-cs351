import { Feather, FontAwesome } from '@expo/vector-icons';
import React, { useRef, useState } from 'react';
import {
  ActivityIndicator,
  KeyboardAvoidingView,
  Platform,

  StyleSheet,
  Text,
  TextInput,
  TouchableOpacity,
  View
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import authService from '../../../../database/authService';

const SignUpScreen = ({ navigation }) => {
  const [firstName, setFirstName] = useState('');
  const [lastName, setLastName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [googleLoading, setGoogleLoading] = useState(false);

  // Input refs for seamless "next" navigation
  const lastNameRef = useRef();
  const emailRef = useRef();
  const passwordRef = useRef();
  const confirmPasswordRef = useRef();

  const handleSignUp = async () => {
    setError('');

    if (!firstName.trim() || !lastName.trim() || !email.trim() || !password || !confirmPassword) {
      setError('Please fill in all fields.');
      return;
    }
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      setError('Please enter a valid email address.');
      return;
    }
    if (password.length < 6) {
      setError('Password must be at least 6 characters.');
      return;
    }
    if (password !== confirmPassword) {
      setError('Passwords do not match.');
      return;
    }

    setLoading(true);
    const result = await authService.signUpWithEmail(email, password, firstName, lastName);
    setLoading(false);

    if (result.success) {
      navigation.navigate("PhoneAndEmailVerificationScreen");
    } else {
      setError(result.error || 'Failed to create account. Try again.');
    }
  };

  const handleGoogleSignUp = async () => {
    setError('');
    setGoogleLoading(true);

    try {
      const result = await authService.signInWithGoogle();

      if (result.success) {
        navigation.navigate('PhoneAndEmailVerificationScreen')
      } else {
        setError(result.error || 'Google sign-up failed. Please try again.');
      }
    } catch (error) {
      console.error('Google sign up error:', error);
      setError('Something went wrong with Google sign-up. Please try again.');
    } finally {
      setGoogleLoading(false);
    }
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <KeyboardAvoidingView
        style={styles.container}
        behavior={Platform.OS === 'ios' ? 'padding' : undefined}
      >
        <Text style={styles.logo}>MyRoomie</Text>

        <View style={styles.header}>
          <Text style={styles.title}>Join the vibe</Text>
          <Text style={styles.subtitle}>Find your perfect roommate match</Text>
        </View>

        {error ? (
          <Text style={styles.errorText}>{error}</Text>
        ) : null}

        {/* First & Last Name Fields */}
        <View style={styles.row}>
          <View style={styles.halfInputWrapper}>
            <Feather name="user" size={18} color="#9CA3AF" style={styles.inputIcon} />
            <TextInput
              placeholder="First name"
              placeholderTextColor="#9CA3AF"
              style={styles.input}
              value={firstName}
              onChangeText={setFirstName}
              autoCapitalize="words"
              returnKeyType="next"
              onSubmitEditing={() => lastNameRef.current.focus()}
              blurOnSubmit={false}
            />
          </View>
          <View style={styles.halfInputWrapper}>
            <Feather name="user" size={18} color="#9CA3AF" style={styles.inputIcon} />
            <TextInput
              ref={lastNameRef}
              placeholder="Last name"
              placeholderTextColor="#9CA3AF"
              style={styles.input}
              value={lastName}
              onChangeText={setLastName}
              autoCapitalize="words"
              returnKeyType="next"
              onSubmitEditing={() => emailRef.current.focus()}
              blurOnSubmit={false}
            />
          </View>
        </View>

        {/* Email */}
        <View style={styles.inputWrapper}>
          <Feather name="mail" size={18} color="#9CA3AF" style={styles.inputIcon} />
          <TextInput
            ref={emailRef}
            placeholder="Email address"
            placeholderTextColor="#9CA3AF"
            style={styles.input}
            keyboardType="email-address"
            autoCapitalize="none"
            value={email}
            onChangeText={setEmail}
            returnKeyType="next"
            onSubmitEditing={() => passwordRef.current.focus()}
            blurOnSubmit={false}
          />
        </View>

        {/* Password */}
        <View style={styles.inputWrapper}>
          <Feather name="lock" size={18} color="#9CA3AF" style={styles.inputIcon} />
          <TextInput
            ref={passwordRef}
            placeholder="Password"
            placeholderTextColor="#9CA3AF"
            style={styles.input}
            secureTextEntry
            value={password}
            onChangeText={setPassword}
            returnKeyType="next"
            onSubmitEditing={() => confirmPasswordRef.current.focus()}
            blurOnSubmit={false}
          />
          <Feather name="eye" size={18} color="#9CA3AF" style={styles.eyeIcon} />
        </View>

        {/* Confirm Password */}
        <View style={styles.inputWrapper}>
          <Feather name="lock" size={18} color="#9CA3AF" style={styles.inputIcon} />
          <TextInput
            ref={confirmPasswordRef}
            placeholder="Confirm password"
            placeholderTextColor="#9CA3AF"
            style={styles.input}
            secureTextEntry
            value={confirmPassword}
            onChangeText={setConfirmPassword}
            returnKeyType="done"
            onSubmitEditing={handleSignUp}
          />
          <Feather name="eye" size={18} color="#9CA3AF" style={styles.eyeIcon} />
        </View>

        <TouchableOpacity
          style={[styles.signUpButton, loading && { opacity: 0.6 }]}
          onPress={handleSignUp}
          disabled={loading}
        >
          <Text style={styles.signUpText}>{loading ? "Creating..." : "Create account"}</Text>
        </TouchableOpacity>

        {/* Divider */}
        <View style={styles.dividerRow}>
          <View style={styles.divider} />
          <Text style={styles.dividerText}>or continue with</Text>
          <View style={styles.divider} />
        </View>

        {/* Google */}
        <TouchableOpacity
          style={[styles.oauthButton, googleLoading && { opacity: 0.6 }]}
          onPress={handleGoogleSignUp}
          disabled={googleLoading}
        >
          {googleLoading ? (
            <ActivityIndicator color="#FFFFFF" size="small" style={styles.oauthIcon} />
          ) : (
            <FontAwesome name="google" size={18} color="#FFFFFF" style={styles.oauthIcon} />
          )}
          <Text style={styles.oauthText}>
            {googleLoading ? 'Signing up...' : 'Continue with Google'}
          </Text>
        </TouchableOpacity>

        <Text style={styles.terms}>
          By signing up, you agree to our <Text style={styles.link}>Terms of Service</Text> and{' '}
          <Text style={styles.link}>Privacy Policy</Text>
        </Text>

        <TouchableOpacity onPress={() => navigation.goBack()}>
          <Text style={styles.loginLink}>
            Already have an account? <Text style={styles.link}>Sign in</Text>
          </Text>
        </TouchableOpacity>
      </KeyboardAvoidingView>
    </SafeAreaView>
  );
};

export default SignUpScreen;

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#111827',
  },
  container: {
    flex: 1,
    padding: 24,
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
  errorText: {
    color: '#EF4444',
    marginBottom: 12,
    textAlign: 'center',
    fontSize: 14,
  },
  row: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  halfInputWrapper: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#1F2937',
    borderRadius: 8,
    paddingHorizontal: 12,
    flex: 0.48,
  },
  inputWrapper: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#1F2937',
    borderRadius: 8,
    paddingHorizontal: 12,
    marginBottom: 12,
  },
  inputIcon: {
    marginRight: 8,
  },
  eyeIcon: {
    marginLeft: 8,
  },
  input: {
    flex: 1,
    paddingVertical: 12,
    color: '#FFFFFF',
    fontSize: 15,
  },
  signUpButton: {
    backgroundColor: '#3B82F6',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
    marginTop: 4,
    marginBottom: 24,
  },
  signUpText: {
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
    backgroundColor: '#374151',
    paddingVertical: 12,
    borderRadius: 8,
    marginBottom: 12,
    paddingHorizontal: 16,
  },
  oauthIcon: {
    marginRight: 12,
  },
  oauthText: {
    color: '#FFFFFF',
    fontSize: 15,
    fontWeight: '500',
  },
  terms: {
    color: '#9CA3AF',
    textAlign: 'center',
    fontSize: 12,
    marginTop: 16,
  },
  link: {
    color: '#60A5FA',
  },
  loginLink: {
    color: '#9CA3AF',
    textAlign: 'center',
    marginTop: 16,
    fontSize: 14,
  },
});
