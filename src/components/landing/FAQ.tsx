import React, { useMemo, useState, useCallback } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { HelpCircle, ChevronDown } from "lucide-react";
import { classNames } from "../../lib/utils";

// Static FAQ data
const faqData = [
  { q: "What do I actually get?", a: "A precise plan, clear trade‑offs, and working artifacts when applicable. If something blocks progress, we show exactly what and how to remove it." },
  { q: "Is my data safe?", a: "Yes. We work under NDA on request and keep runs deterministic and auditable. You control what's shared." },
  { q: "What if the problem is unsolvable?", a: "We show proof why it's blocked and what would need to change. No wasted cycles on dead ends." },
];

const FAQ = React.memo(() => {
  const qa = useMemo(() => faqData, []);
  const [expandedIndex, setExpandedIndex] = useState<number | null>(null);

  const toggleQuestion = useCallback((index: number) => {
    setExpandedIndex(prev => prev === index ? null : index);
  }, []);

  return (
    <section className="border-t border-white/10 px-4 py-24 sm:px-6 lg:px-8">
      <div className="mx-auto max-w-3xl">
        <motion.div
          initial={{ opacity: 0 }}
          whileInView={{ opacity: 1 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="text-center mb-12"
        >
          <h2 className="text-3xl font-bold md:text-5xl">
            <span className="bg-gradient-to-r from-white to-white/60 bg-clip-text text-transparent">
              Common questions
            </span>
          </h2>
        </motion.div>

        <div className="space-y-4">
          {qa.map((item, index) => (
            <motion.div
              key={index}
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: index * 0.1, duration: 0.5 }}
              viewport={{ once: true }}
              className="rounded-2xl border border-white/10 bg-white/5 overflow-hidden"
            >
              <button
                onClick={() => toggleQuestion(index)}
                className="flex w-full items-center justify-between p-6 text-left hover:bg-white/10 transition-colors"
                aria-expanded={expandedIndex === index}
                aria-controls={`faq-answer-${index}`}
              >
                <div className="flex items-center gap-3">
                  <HelpCircle className="h-5 w-5 text-white/60" />
                  <span className="font-medium">{item.q}</span>
                </div>
                <ChevronDown
                  className={classNames(
                    "h-4 w-4 text-white/60 transition-transform",
                    expandedIndex === index && "rotate-180"
                  )}
                />
              </button>

              <AnimatePresence>
                {expandedIndex === index && (
                  <motion.div
                    id={`faq-answer-${index}`}
                    initial={{ height: 0, opacity: 0 }}
                    animate={{ height: "auto", opacity: 1 }}
                    exit={{ height: 0, opacity: 0 }}
                    transition={{ duration: 0.3 }}
                    className="overflow-hidden"
                  >
                    <p className="px-6 pb-6 text-sm text-white/70">
                      {item.a}
                    </p>
                  </motion.div>
                )}
              </AnimatePresence>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
});

FAQ.displayName = "FAQ";

export default FAQ;
