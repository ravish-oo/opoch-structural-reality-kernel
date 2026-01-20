import { useState } from "react";
import { z } from "zod";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { supabase, isSupabaseConfigured } from "../lib/supabase";
import { sendApplicationReceivedEmail } from "../lib/email";
import { trackLeadSubmitted } from "../lib/analytics";
import { Dialog, DialogContent, DialogHeader, DialogTitle } from "./ui/dialog";
import { Button } from "./ui/button";
import { Input } from "./ui/input";
import { Textarea } from "./ui/textarea";

const Schema = z.object({
  name: z.string().min(2, "Name must be at least 2 characters"),
  email: z.string().email("Invalid email address"),
  phone: z.string().optional(),
  designation: z.string().min(2, "Designation must be at least 2 characters"),
  organization: z.string().min(2, "Organization must be at least 2 characters"),
  reason: z.string().min(10, "Please tell us why you're interested (min 10 characters)"),
  research_query: z.string().optional(),
  source: z.string().optional(),
});

type FormValues = z.infer<typeof Schema>;

interface ApplyModalProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  source?: string;
}

export default function ApplyModal({ open, onOpenChange, source = "hero" }: ApplyModalProps) {
  const [isSuccess, setIsSuccess] = useState(false);
  
  const {
    register,
    handleSubmit,
    reset,
    formState: { errors, isSubmitting },
  } = useForm<FormValues>({
    resolver: zodResolver(Schema),
    defaultValues: { source },
  });

  async function onSubmit(values: FormValues) {
    try {
      let leadId: string | undefined;
      
      if (isSupabaseConfigured()) {
        const { data, error } = await supabase.from("leads").insert({
          ...values,
          utm: {}, // Add UTM tracking if needed
        }).select('id').single();
        
        if (!error && data) {
          leadId = data.id;
          
          // Send welcome email (fire and forget)
          sendApplicationReceivedEmail(
            values.name,
            values.email,
            values.research_query || '',
            leadId
          );
          
          // Track analytics
          trackLeadSubmitted(values.source || 'unknown');
          
          setIsSuccess(true);
          setTimeout(() => {
            onOpenChange(false);
            reset();
            setIsSuccess(false);
          }, 2000);
        } else {
          console.error("Failed to submit lead:", error);
        }
      } else {
        // Just show success even if not connected to database
        console.log("Lead submission (not saved to DB):", values);
        setIsSuccess(true);
        setTimeout(() => {
          onOpenChange(false);
          reset();
          setIsSuccess(false);
        }, 2000);
      }
    } catch (err) {
      console.error("Failed to submit lead:", err);
    }
  }

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-md rounded-2xl border-white/10 bg-[#0B0F1A] text-white">
        <DialogHeader>
          <DialogTitle className="text-xl font-semibold">Apply for access</DialogTitle>
        </DialogHeader>
        
        {isSuccess ? (
          <div className="py-8 text-center">
            <div className="mb-4 text-4xl">✓</div>
            <p className="text-lg font-medium">Application received!</p>
            <p className="mt-2 text-sm text-white/70">We'll reach out within 24 hours.</p>
          </div>
        ) : (
          <form onSubmit={handleSubmit(onSubmit)} className="mt-4 grid gap-3">
            <div>
              <Input
                placeholder="Full name"
                className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                {...register("name")}
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
                {...register("email")}
              />
              {errors.email && (
                <p className="mt-1 text-xs text-red-400">{errors.email.message}</p>
              )}
            </div>

            <div>
              <Input
                placeholder="Phone (optional)"
                className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                {...register("phone")}
              />
            </div>

            <div>
              <Input
                placeholder="Your designation"
                className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                {...register("designation")}
              />
              {errors.designation && (
                <p className="mt-1 text-xs text-red-400">{errors.designation.message}</p>
              )}
            </div>

            <div>
              <Input
                placeholder="Organization"
                className="border-white/10 bg-white/5 text-white placeholder:text-white/50"
                {...register("organization")}
              />
              {errors.organization && (
                <p className="mt-1 text-xs text-red-400">{errors.organization.message}</p>
              )}
            </div>

            <div>
              <Textarea
                placeholder="Why do you want access to Opoch? What problem are you solving?"
                rows={3}
                className="border-white/10 bg-white/5 text-white placeholder:text-white/50 resize-none"
                {...register("reason")}
              />
              {errors.reason && (
                <p className="mt-1 text-xs text-red-400">{errors.reason.message}</p>
              )}
            </div>

            <div>
              <Textarea
                placeholder="Your research question or technical challenge (optional)"
                rows={3}
                className="border-white/10 bg-white/5 text-white placeholder:text-white/50 resize-none"
                {...register("research_query")}
              />
            </div>

            <input type="hidden" {...register("source")} />

            <div className="mt-4 flex items-center justify-between">
              <span className="text-xs text-white/50">Privacy-first. No spam.</span>
              <Button
                type="submit"
                disabled={isSubmitting}
                className="rounded-2xl bg-white text-black hover:bg-white/90 disabled:opacity-50"
              >
                {isSubmitting ? "Submitting..." : "Submit application"}
              </Button>
            </div>
          </form>
        )}
      </DialogContent>
    </Dialog>
  );
}