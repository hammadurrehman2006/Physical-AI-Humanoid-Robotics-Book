// book/src/components/auth/SignUpForm.tsx
import React, { useState } from 'react';
import { useAuth } from '../../auth/hooks/useAuth';
import { signupSchema, SignupData } from '../../auth/validators/schemas';
import { AUTH_ERRORS } from '../../auth/utils/errors';

interface SignUpFormProps {
  onSuccess?: () => void;
}

const SignUpForm: React.FC<SignUpFormProps> = ({ onSuccess }) => {
  const { signUp } = useAuth();
  const [formData, setFormData] = useState<SignupData>({
    email: '',
    password: '',
    confirmPassword: '',
    software_background: 'Beginner',
    hardware_background: 'No Experience',
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState(false);
  const [generalError, setGeneralError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));

    // Clear field error when user starts typing
    if (errors[name]) {
      setErrors(prev => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }

    // Clear general error when user starts typing
    if (generalError) {
      setGeneralError(null);
    }
  };

  const validateForm = (): boolean => {
    try {
      signupSchema.parse(formData);
      setErrors({});
      return true;
    } catch (err: any) {
      const fieldErrors: Record<string, string> = {};
      if (err.errors) {
        err.errors.forEach((error: any) => {
          if (error.path) {
            fieldErrors[error.path[0]] = error.message;
          }
        });
      }
      setErrors(fieldErrors);
      return false;
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setLoading(true);
    setGeneralError(null);

    try {
      const result = await signUp({
        email: formData.email,
        password: formData.password,
        user: {
          software_background: formData.software_background,
          hardware_background: formData.hardware_background,
        },
      });

      if (result?.error) {
        if (result.error.code === 'USER_EXISTS_WITH_SAME_EMAIL') {
          setErrors({ email: AUTH_ERRORS.DUPLICATE_EMAIL.message });
        } else {
          setGeneralError(result.error.message || 'An error occurred during registration');
        }
      } else if (result?.data) {
        // Registration successful
        if (onSuccess) {
          onSuccess();
        }
      }
    } catch (error: any) {
      setGeneralError(error.message || 'An unexpected error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="space-y-4">
      {generalError && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded relative" role="alert">
          {generalError}
        </div>
      )}

      <div>
        <label htmlFor="email" className="block text-sm font-medium text-gray-700 mb-1">
          Email Address <span className="text-red-500">*</span>
        </label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
          aria-label="Email address"
          className={`w-full px-3 py-2 border rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 ${
            errors.email ? 'border-red-500' : 'border-gray-300'
          }`}
          aria-invalid={!!errors.email}
          aria-describedby={errors.email ? 'email-error' : undefined}
        />
        {errors.email && (
          <p id="email-error" className="mt-1 text-sm text-red-600" role="alert">
            {errors.email}
          </p>
        )}
      </div>

      <div>
        <label htmlFor="password" className="block text-sm font-medium text-gray-700 mb-1">
          Password <span className="text-red-500">*</span>
        </label>
        <input
          type="password"
          id="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          aria-label="Password"
          className={`w-full px-3 py-2 border rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 ${
            errors.password ? 'border-red-500' : 'border-gray-300'
          }`}
          aria-invalid={!!errors.password}
          aria-describedby={errors.password ? 'password-error' : undefined}
        />
        {errors.password && (
          <p id="password-error" className="mt-1 text-sm text-red-600" role="alert">
            {errors.password}
          </p>
        )}
      </div>

      <div>
        <label htmlFor="confirmPassword" className="block text-sm font-medium text-gray-700 mb-1">
          Confirm Password <span className="text-red-500">*</span>
        </label>
        <input
          type="password"
          id="confirmPassword"
          name="confirmPassword"
          value={formData.confirmPassword}
          onChange={handleChange}
          aria-label="Confirm password"
          className={`w-full px-3 py-2 border rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 ${
            errors.confirmPassword ? 'border-red-500' : 'border-gray-300'
          }`}
          aria-invalid={!!errors.confirmPassword}
          aria-describedby={errors.confirmPassword ? 'confirm-password-error' : undefined}
        />
        {errors.confirmPassword && (
          <p id="confirm-password-error" className="mt-1 text-sm text-red-600" role="alert">
            {errors.confirmPassword}
          </p>
        )}
      </div>

      <div>
        <label htmlFor="software_background" className="block text-sm font-medium text-gray-700 mb-1">
          Software Experience Level <span className="text-red-500">*</span>
        </label>
        <select
          id="software_background"
          name="software_background"
          value={formData.software_background}
          onChange={handleChange}
          aria-label="Software experience level"
          className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
        >
          <option value="Beginner">Beginner</option>
          <option value="Intermediate">Intermediate</option>
          <option value="Advanced">Advanced</option>
          <option value="Expert">Expert</option>
        </select>
      </div>

      <div>
        <label htmlFor="hardware_background" className="block text-sm font-medium text-gray-700 mb-1">
          Hardware Experience Level <span className="text-red-500">*</span>
        </label>
        <select
          id="hardware_background"
          name="hardware_background"
          value={formData.hardware_background}
          onChange={handleChange}
          aria-label="Hardware experience level"
          className="w-full px-3 py-2 border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
        >
          <option value="No Experience">No Experience</option>
          <option value="Basic Knowledge">Basic Knowledge</option>
          <option value="Hands-on Experience">Hands-on Experience</option>
          <option value="Professional Experience">Professional Experience</option>
        </select>
      </div>

      <button
        type="submit"
        disabled={loading}
        className={`w-full flex justify-center py-2 px-4 border border-transparent rounded-md shadow-sm text-sm font-medium text-white ${
          loading
            ? 'bg-blue-400 cursor-not-allowed'
            : 'bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500'
        }`}
        aria-busy={loading}
      >
        {loading ? 'Creating account...' : 'Sign Up'}
      </button>
    </form>
  );
};

export default SignUpForm;