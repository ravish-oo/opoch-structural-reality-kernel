import { useState, useEffect, useRef, useMemo, useCallback } from "react";
import { z } from "zod";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { motion, AnimatePresence } from "framer-motion";
import { isSupabaseConfigured } from "../lib/supabase";
import { trackLeadSubmitted } from "../lib/analytics";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
} from "./ui/dialog";
import { Button } from "./ui/button";
import { Input } from "./ui/input";
import { Textarea } from "./ui/textarea";
import { CheckCircle } from "lucide-react";
import { toastError } from "../hooks/use-toast";
import type { Lead, ApiError } from "../types";
import { PhoneInput } from "./ui/phone-input";
import type { Value } from "react-phone-number-input";
import "../styles/phone-input.css";

const createApplyFormSchema = (hasCompletedProfile: boolean) => z.object({
  name: hasCompletedProfile ? z.string().optional() : z.string().min(2, "Name must be at least 2 characters"),
  email: hasCompletedProfile ? z.string().optional() : z.string().email("Invalid email address"),
  phone: z.string().optional(),
  designation: hasCompletedProfile ? z.string().optional() : z.string().min(2, "Designation must be at least 2 characters"),
  organization: hasCompletedProfile ? z.string().optional() : z.string().min(2, "Organization must be at least 2 characters"),
  reason: hasCompletedProfile ? z.string().optional() : z.string().min(10, "Please tell us why you're interested (min 10 characters)"),
  research_query: hasCompletedProfile 
    ? z.string().min(20, "Please describe your technical challenge (min 20 characters)")
    : z.string().optional(),
  source: z.string().optional(),
});

type ApplyFormValues = z.infer<ReturnType<typeof createApplyFormSchema>>;

interface ApplyModalProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  source?: string;
  onSuccess?: (lead: Lead) => void;
  user?: any;
}

// Local storage key for form data
const FORM_STORAGE_KEY = 'opoch.apply.form';

export function ApplyModal({
  open,
  onOpenChange,
  source = "website",
  onSuccess,
  user
}: ApplyModalProps) {
  const [isSuccess, setIsSuccess] = useState(false);
  const [submitError, setSubmitError] = useState<string | null>(null);
  const [userProfile, setUserProfile] = useState<any>(null);
  const [phoneValue, setPhoneValue] = useState<Value | undefined>(undefined);
  const [isInitialized, setIsInitialized] = useState(false);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  const saveTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  // Load form data from localStorage
  const loadFormData = useCallback((): Partial<ApplyFormValues> | null => {
    try {
      const saved = localStorage.getItem(FORM_STORAGE_KEY);
      return saved ? JSON.parse(saved) : null;
    } catch (error) {
      console.error('Failed to load form data:', error);
      return null;
    }
  }, []);

  // Save form data to localStorage (excluding research_query)
  const saveFormData = useCallback((data: Partial<ApplyFormValues>) => {
    try {
      const dataToSave = {
        name: data.name,
        email: data.email,
        phone: data.phone,
        designation: data.designation,
        organization: data.organization,
        reason: data.reason,
        // Explicitly exclude research_query
      };
      localStorage.setItem(FORM_STORAGE_KEY, JSON.stringify(dataToSave));
    } catch (error) {
      console.error('Failed to save form data:', error);
    }
  }, []);
  
  // Create a stable schema
  const formSchema = useMemo(() => {
    return createApplyFormSchema(userProfile?.has_completed_profile || false);
  }, [userProfile?.has_completed_profile]);

  // Get initial values including saved data
  const getInitialValues = useCallback(() => {
    const savedData = loadFormData();
    return {
      source,
      name: savedData?.name || user?.user_metadata?.full_name || user?.user_metadata?.name || '',
      email: savedData?.email || user?.email || '',
      phone: savedData?.phone || '',
      designation: savedData?.designation || '',
      organization: savedData?.organization || '',
      reason: savedData?.reason || '',
      research_query: '',
    };
  }, [source, user, loadFormData]);

  const {
    register,
    handleSubmit,
    reset,
    setValue,
    getValues,
    formState: { errors, isSubmitting },
  } = useForm<ApplyFormValues>({
    resolver: zodResolver(formSchema),
    defaultValues: getInitialValues(),
  });

  // Initialize form when modal opens
  useEffect(() => {
    if (open && !isInitialized) {
      const initialValues = getInitialValues();
      
      // Reset form with initial values
      reset(initialValues);
      
      // Set phone value
      if (initialValues.phone) {
        setPhoneValue(initialValues.phone as Value);
      }
      
      setIsInitialized(true);
    } else if (!open) {
      setIsInitialized(false);
    }
  }, [open, isInitialized, reset, getInitialValues]);

  // Fetch user profile
  useEffect(() => {
    async function fetchProfile() {
      if (open && user?.id && !userProfile) {
        try {
          const response = await fetch(`/api/get-profile?userId=${user.id}`);
          const result = await response.json();
          if (result.success && result.profile) {
            setUserProfile(result.profile);
          }
        } catch (error) {
          console.error('Failed to fetch profile:', error);
        }
      }
    }
    fetchProfile();
  }, [open, user?.id, userProfile]);

  // Debounced save function
  const debouncedSave = useCallback((fieldName: string, value: any) => {
    if (saveTimeoutRef.current) {
      clearTimeout(saveTimeoutRef.current);
    }
    
    saveTimeoutRef.current = setTimeout(() => {
      const currentValues = getValues();
      saveFormData({
        ...currentValues,
        [fieldName]: value,
        phone: fieldName === 'phone' ? value : currentValues.phone
      });
    }, 500);
  }, [getValues, saveFormData]);

  // Cleanup timeouts
  useEffect(() => {
    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
      if (saveTimeoutRef.current) {
        clearTimeout(saveTimeoutRef.current);
      }
    };
  }, []);

  async function onSubmit(values: ApplyFormValues) {
    console.log('🔵 Form submission started', values);
    setSubmitError(null);
    
    try {
      const leadData = {
        ...values,
        phone: phoneValue || values.phone || '',
        reason: values.reason || `Interested in technical problem solving and ${values.research_query ? 'has specific research query' : 'general inquiry'}`,
        source: values.source || source,
        user_id: user?.id || null
      };

      if (isSupabaseConfigured()) {
        console.log('🔵 Calling Supabase API');
        const { data, error } = await callSupabaseAPI(leadData);
        
        if (error) {
          throw error;
        }
        
        if (data) {
          console.log('✅ Lead created successfully:', data);
          setIsSuccess(true);
          
          await trackLeadSubmitted(leadData.source || source);
          
          onSuccess?.(data);
          
          // Clear saved form data after successful submission
          localStorage.removeItem(FORM_STORAGE_KEY);
          
          timeoutRef.current = setTimeout(() => {
            console.log('🔵 Resetting form state');
            onOpenChange(false);
            reset();
            setPhoneValue(undefined);
            setIsSuccess(false);
            setSubmitError(null);
            timeoutRef.current = null;
          }, 3000);
        } else {
          console.error('🔴 No data returned from Supabase insert');
        }
      } else {
        console.log('🟡 Supabase not configured, using API endpoint');
        
        const response = await fetch('/api/submit-lead', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(leadData),
        });

        if (!response.ok) {
          const error = await response.json() as ApiError;
          throw new Error(error.message || `Failed to submit: ${response.statusText}`);
        }

        const result = await response.json() as Lead;
        setIsSuccess(true);
        
        await trackLeadSubmitted(leadData.source || source);
        
        onSuccess?.(result);
        
        // Clear saved form data after successful submission
        localStorage.removeItem(FORM_STORAGE_KEY);
        
        timeoutRef.current = setTimeout(() => {
          onOpenChange(false);
          reset();
          setPhoneValue(undefined);
          setIsSuccess(false);
          timeoutRef.current = null;
        }, 3000);
      }
    } catch (err) {
      console.error('🔴 Unexpected error in form submission:', err);
      const errorMessage = err instanceof Error ? err.message : 'Failed to submit application';
      setSubmitError(errorMessage);
      toastError(errorMessage);
    }
  }

  async function callSupabaseAPI(leadData: any) {
    const { supabase } = await import('../lib/supabase');
    return await supabase
      .from('leads')
      .insert([leadData])
      .select()
      .single();
  }

  const handlePhoneChange = useCallback((value: Value | undefined) => {
    setPhoneValue(value);
    setValue('phone', value || '', { shouldValidate: false, shouldDirty: true });
    debouncedSave('phone', value || '');
  }, [setValue, debouncedSave]);

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent 
        className="max-w-md rounded-2xl border-white/10 bg-[#0B0F1A] text-white" 
        aria-describedby="apply-description"
        onOpenAutoFocus={(e) => e.preventDefault()}
      >
        <DialogHeader>
          <DialogTitle className="text-xl font-semibold">
            {(userProfile?.has_completed_profile || false) ? 'Submit your query' : 'Apply for access'}
          </DialogTitle>
          <p id="apply-description" className="sr-only">
            {(userProfile?.has_completed_profile || false) 
              ? 'Submit your technical query to get expert guidance.'
              : 'Fill out this form to apply for Opoch membership and get guidance on your technical challenges.'}
          </p>
        </DialogHeader>
        
        <AnimatePresence mode="wait">
          {isSuccess ? (
            <motion.div
              key="success"
              initial={{ opacity: 0, scale: 0.95 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.95 }}
              className="flex flex-col items-center justify-center py-8 text-center"
            >
              <motion.div
                initial={{ scale: 0 }}
                animate={{ scale: 1 }}
                transition={{ delay: 0.1, type: "spring", stiffness: 500, damping: 30 }}
              >
                <CheckCircle className="h-16 w-16 text-emerald-400" />
              </motion.div>
              
              <motion.h3
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.2 }}
                className="mt-4 text-lg font-semibold"
              >
                {userProfile?.has_completed_profile ? 'Query submitted!' : 'Application received!'}
              </motion.h3>
              
              <motion.p
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.3 }}
                className="mt-2 text-sm text-white/60"
              >
                {userProfile?.has_completed_profile 
                  ? "We'll review your technical query and get back to you within 24-48 hours."
                  : "We'll review your application and get back to you within 24-48 hours. Check your email for confirmation."}
              </motion.p>
            </motion.div>
          ) : (
            <motion.form
              key="form"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              onSubmit={handleSubmit(onSubmit)}
              className="space-y-4"
            >
              {!(userProfile?.has_completed_profile || false) && (
                <>
                  <div>
                    <Input
                      placeholder="Your name"
                      className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                      {...register("name", {
                        onBlur: (e) => debouncedSave('name', e.target.value)
                      })}
                    />
                    {errors.name && (
                      <p className="mt-1 text-xs text-red-400">{errors.name.message}</p>
                    )}
                  </div>

                  <div>
                    <Input
                      placeholder="Email"
                      type="email"
                      className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                      {...register("email", {
                        onBlur: (e) => debouncedSave('email', e.target.value)
                      })}
                    />
                    {errors.email && (
                      <p className="mt-1 text-xs text-red-400">{errors.email.message}</p>
                    )}
                  </div>

                  <div>
                    <PhoneInput
                      value={phoneValue}
                      onChange={handlePhoneChange}
                      placeholder="Phone (optional)"
                      className="border-white/10 bg-white/5"
                      error={!!errors.phone}
                    />
                    {errors.phone && (
                      <p className="mt-1 text-xs text-red-400">{errors.phone.message}</p>
                    )}
                  </div>

                  <div>
                    <Input
                      placeholder="Your designation"
                      className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                      {...register("designation", {
                        onBlur: (e) => debouncedSave('designation', e.target.value)
                      })}
                    />
                    {errors.designation && (
                      <p className="mt-1 text-xs text-red-400">{errors.designation.message}</p>
                    )}
                  </div>

                  <div>
                    <Input
                      placeholder="Organization"
                      className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                      {...register("organization", {
                        onBlur: (e) => debouncedSave('organization', e.target.value)
                      })}
                    />
                    {errors.organization && (
                      <p className="mt-1 text-xs text-red-400">{errors.organization.message}</p>
                    )}
                  </div>

                  <div>
                    <Textarea
                      placeholder="Why do you want access to Opoch? What problem are you solving?"
                      className="min-h-[100px] border-white/10 bg-white/5 text-white placeholder:text-white/50"
                      {...register("reason", {
                        onBlur: (e) => debouncedSave('reason', e.target.value)
                      })}
                    />
                    {errors.reason && (
                      <p className="mt-1 text-xs text-red-400">{errors.reason.message}</p>
                    )}
                  </div>
                </>
              )}

              {(userProfile?.has_completed_profile || false) && (
                <div className="mb-4">
                  <p className="text-sm text-white/60 mb-2">Your details are already on file:</p>
                  <p className="text-sm text-white/80">
                    {userProfile.full_name || user?.user_metadata?.full_name || 'User'} • {userProfile.designation || 'Member'} at {userProfile.organization || 'Organization'}
                  </p>
                  <button
                    type="button"
                    onClick={() => setUserProfile({ ...userProfile, has_completed_profile: false })}
                    className="text-xs text-white/60 underline mt-2"
                  >
                    Edit my details
                  </button>
                </div>
              )}

              <div>
                <Textarea
                  placeholder={(userProfile?.has_completed_profile || false) 
                    ? "Describe your research question or technical challenge" 
                    : "Your research question or technical challenge (optional)"}
                  className="min-h-[100px] border-white/10 bg-white/5 text-white placeholder:text-white/50"
                  {...register("research_query")}
                />
                {(userProfile?.has_completed_profile || false) && errors.research_query && (
                  <p className="mt-1 text-xs text-red-400">{errors.research_query.message}</p>
                )}
              </div>

              {submitError && (
                <motion.p
                  initial={{ opacity: 0, y: -10 }}
                  animate={{ opacity: 1, y: 0 }}
                  className="text-sm text-red-400"
                >
                  {submitError}
                </motion.p>
              )}

              <div className="flex gap-4 pt-2">
                <p className="flex-1 text-xs text-white/40">
                  Privacy-first. No spam.
                </p>
                <Button
                  type="submit"
                  disabled={isSubmitting}
                  className="bg-white text-black hover:bg-white/90"
                >
                  {isSubmitting ? 'Submitting...' : 'Submit application'}
                </Button>
              </div>
            </motion.form>
          )}
        </AnimatePresence>
      </DialogContent>
    </Dialog>
  );
}