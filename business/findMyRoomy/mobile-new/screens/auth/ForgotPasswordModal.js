import React, { useState, useRef, useEffect } from 'react';
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  StyleSheet,
  Modal,
  Animated,
  ActivityIndicator,
  Dimensions,
} from 'react-native';
import { Feather } from '@expo/vector-icons';
import authService from '../../database/authService';

const { width } = Dimensions.get('window');

const ForgotPasswordModal = ({ visible, onClose }) => {
  const [email, setEmail] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);
  
  // Animation values
  const overlayOpacity = useRef(new Animated.Value(0)).current;
  const modalScale = useRef(new Animated.Value(0.8)).current;
  const modalOpacity = useRef(new Animated.Value(0)).current;

  useEffect(() => {
    if (visible) {
      // Reset states when modal opens
      setEmail('');
      setError('');
      setSuccess(false);
      setLoading(false);
      
      // Animate modal in
      Animated.parallel([
        Animated.timing(overlayOpacity, {
          toValue: 1,
          duration: 200,
          useNativeDriver: true,
        }),
        Animated.timing(modalOpacity, {
          toValue: 1,
          duration: 200,
          useNativeDriver: true,
        }),
        Animated.spring(modalScale, {
          toValue: 1,
          tension: 100,
          friction: 8,
          useNativeDriver: true,
        }),
      ]).start();
    } else {
      // Animate modal out
      Animated.parallel([
        Animated.timing(overlayOpacity, {
          toValue: 0,
          duration: 150,
          useNativeDriver: true,
        }),
        Animated.timing(modalOpacity, {
          toValue: 0,
          duration: 150,
          useNativeDriver: true,
        }),
        Animated.timing(modalScale, {
          toValue: 0.8,
          duration: 150,
          useNativeDriver: true,
        }),
      ]).start();
    }
  }, [visible]);

  const handleSendResetEmail = async () => {
    // Basic validation
    if (!email.trim()) {
      setError('Please enter your email address');
      return;
    }

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      setError('Please enter a valid email address');
      return;
    }

    setError('');
    setLoading(true);

    try {
      const result = await authService.resetPassword(email);
      
      if (result.success) {
        setSuccess(true);
        setError('');
      } else {
        setError(result.error || 'Failed to send reset email. Please try again.');
      }
    } catch (error) {
      console.error('Reset password error:', error);
      setError('Something went wrong. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleClose = () => {
    if (!loading) {
      onClose();
    }
  };

  const handleOverlayPress = () => {
    if (!loading) {
      onClose();
    }
  };

  if (!visible) return null;

  return (
    <Modal
      transparent
      visible={visible}
      animationType="none"
      onRequestClose={handleClose}
    >
      <Animated.View 
        style={[styles.overlay, { opacity: overlayOpacity }]}
      >
        <TouchableOpacity 
          style={styles.overlayTouchable}
          activeOpacity={1}
          onPress={handleOverlayPress}
        >
          <Animated.View 
            style={[
              styles.modalContainer,
              {
                opacity: modalOpacity,
                transform: [{ scale: modalScale }],
              },
            ]}
          >
            <TouchableOpacity activeOpacity={1} onPress={() => {}}>
              {/* Modal Header */}
              <View style={styles.header}>
                <Text style={styles.title}>Reset Password</Text>
                <TouchableOpacity 
                  onPress={handleClose}
                  style={styles.closeButton}
                  disabled={loading}
                >
                  <Feather name="x" size={24} color="#9CA3AF" />
                </TouchableOpacity>
              </View>

              {/* Modal Content */}
              <View style={styles.content}>
                {!success ? (
                  <>
                    <Text style={styles.description}>
                      Enter your email address and we'll send you a link to reset your password.
                    </Text>

                    {/* Error Message */}
                    {error ? (
                      <View style={styles.errorContainer}>
                        <Text style={styles.errorText}>{error}</Text>
                      </View>
                    ) : null}

                    {/* Email Input */}
                    <View style={[
                      styles.inputWrapper, 
                      error && styles.inputError
                    ]}>
                      <Feather 
                        name="mail" 
                        size={18} 
                        color="#9CA3AF" 
                        style={styles.inputIcon} 
                      />
                      <TextInput
                        placeholder="Email address"
                        placeholderTextColor="#9CA3AF"
                        style={styles.input}
                        keyboardType="email-address"
                        autoCapitalize="none"
                        value={email}
                        onChangeText={(text) => {
                          setEmail(text);
                          if (error) setError('');
                        }}
                        editable={!loading}
                        autoFocus={true}
                      />
                    </View>

                    {/* Action Buttons */}
                    <View style={styles.buttonContainer}>
                      <TouchableOpacity 
                        style={[styles.sendButton, loading && styles.sendButtonDisabled]}
                        onPress={handleSendResetEmail}
                        disabled={loading}
                      >
                        {loading ? (
                          <ActivityIndicator color="#FFFFFF" size="small" />
                        ) : (
                          <Text style={styles.sendButtonText}>Send Reset Link</Text>
                        )}
                      </TouchableOpacity>

                      <TouchableOpacity 
                        style={[styles.cancelButton, loading && styles.cancelButtonDisabled]}
                        onPress={handleClose}
                        disabled={loading}
                      >
                        <Text style={[styles.cancelButtonText, loading && styles.disabledText]}>
                          Cancel
                        </Text>
                      </TouchableOpacity>
                    </View>
                  </>
                ) : (
                  // Success State
                  <View style={styles.successContainer}>
                    <View style={styles.successIconContainer}>
                      <Feather name="check-circle" size={48} color="#10B981" />
                    </View>
                    <Text style={styles.successTitle}>Email Sent!</Text>
                    <Text style={styles.successDescription}>
                      We've sent a password reset link to {email}. 
                      Please check your email and follow the instructions to reset your password.
                    </Text>
                    <TouchableOpacity 
                      style={styles.doneButton}
                      onPress={handleClose}
                    >
                      <Text style={styles.doneButtonText}>Done</Text>
                    </TouchableOpacity>
                  </View>
                )}
              </View>
            </TouchableOpacity>
          </Animated.View>
        </TouchableOpacity>
      </Animated.View>
    </Modal>
  );
};

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.6)',
    justifyContent: 'center',
    alignItems: 'center',
  },
  overlayTouchable: {
    flex: 1,
    width: '100%',
    justifyContent: 'center',
    alignItems: 'center',
    paddingHorizontal: 24,
  },
  modalContainer: {
    backgroundColor: '#1F2937',
    borderRadius: 16,
    width: '100%',
    maxWidth: 400,
    borderWidth: 1,
    borderColor: 'rgba(59, 130, 246, 0.2)',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 10 },
    shadowOpacity: 0.3,
    shadowRadius: 20,
    elevation: 20,
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingTop: 20,
    paddingBottom: 16,
    borderBottomWidth: 1,
    borderBottomColor: 'rgba(55, 65, 81, 0.5)',
  },
  title: {
    fontSize: 20,
    fontWeight: '700',
    color: '#FFFFFF',
  },
  closeButton: {
    padding: 4,
  },
  content: {
    padding: 20,
  },
  description: {
    fontSize: 14,
    color: '#9CA3AF',
    lineHeight: 20,
    marginBottom: 20,
  },
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
    marginBottom: 20,
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
  buttonContainer: {
    gap: 12,
  },
  sendButton: {
    backgroundColor: '#3B82F6',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
    shadowColor: '#3B82F6',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 8,
    elevation: 8,
  },
  sendButtonDisabled: {
    backgroundColor: '#374151',
    shadowOpacity: 0,
    elevation: 0,
  },
  sendButtonText: {
    color: '#FFFFFF',
    fontSize: 15,
    fontWeight: '600',
  },
  cancelButton: {
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
    backgroundColor: 'transparent',
    borderWidth: 1,
    borderColor: 'rgba(55, 65, 81, 0.8)',
  },
  cancelButtonDisabled: {
    opacity: 0.5,
  },
  cancelButtonText: {
    color: '#9CA3AF',
    fontSize: 15,
    fontWeight: '500',
  },
  disabledText: {
    opacity: 0.5,
  },
  successContainer: {
    alignItems: 'center',
  },
  successIconContainer: {
    marginBottom: 16,
  },
  successTitle: {
    fontSize: 20,
    fontWeight: '700',
    color: '#FFFFFF',
    marginBottom: 12,
  },
  successDescription: {
    fontSize: 14,
    color: '#9CA3AF',
    lineHeight: 20,
    textAlign: 'center',
    marginBottom: 24,
  },
  doneButton: {
    backgroundColor: '#10B981',
    paddingVertical: 14,
    paddingHorizontal: 32,
    borderRadius: 8,
    alignItems: 'center',
    minWidth: 120,
  },
  doneButtonText: {
    color: '#FFFFFF',
    fontSize: 15,
    fontWeight: '600',
  },
});

export default ForgotPasswordModal;