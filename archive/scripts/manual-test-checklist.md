# Manual Test Checklist for Opoch

## Pre-Deployment Testing

### 1. Build Test
- [ ] Run `npm run build` - should complete without errors
- [ ] Check build size is reasonable (< 5MB)

### 2. Homepage Tests
- [ ] Page loads without errors
- [ ] All sections visible (Hero, Outcomes, Process, etc.)
- [ ] Navigation links work
- [ ] Apply button opens modal
- [ ] Mobile responsive design works

### 3. Form Tests

#### Apply Modal
- [ ] Modal opens when Apply button clicked
- [ ] Form validation works (try submitting empty)
- [ ] Required fields show error messages
- [ ] Form submission works (check console for logs)
- [ ] Success message appears after submission
- [ ] Modal closes after success

#### AskBox (RBT Section)
- [ ] Form validation for email
- [ ] Query submission works
- [ ] Success feedback shown

### 4. Navigation Tests
- [ ] Moonshots page loads (`/moonshots`)
- [ ] Individual moonshot pages load
- [ ] Updates page loads (`/updates`)
- [ ] Back navigation works
- [ ] Footer links work

### 5. Moonshots Page
- [ ] All 15 moonshots display
- [ ] Domain filters work (AI/ML, Data, etc.)
- [ ] Status filters work
- [ ] Click on moonshot navigates to detail page
- [ ] Apply button on detail page works

### 6. Updates Page
- [ ] Page loads with sample updates
- [ ] Newsletter signup form present
- [ ] Update cards display correctly
- [ ] Click on update shows detail (if implemented)

### 7. Auth Tests (if Supabase configured)
- [ ] Sign in button appears (if implemented)
- [ ] OAuth redirects work
- [ ] User menu appears when logged in
- [ ] Protected routes redirect when not authenticated

### 8. Admin Pages (if authenticated as admin)
- [ ] `/admin/emails` accessible
- [ ] `/admin/leads` accessible
- [ ] Data displays correctly

### 9. PWA Tests
- [ ] Service worker registers (check DevTools > Application)
- [ ] Manifest loads (check DevTools > Application)
- [ ] Works offline (disconnect internet and reload)

### 10. SEO & Performance
- [ ] Check meta tags in page source
- [ ] Sitemap accessible at `/sitemap.xml`
- [ ] Robots.txt accessible at `/robots.txt`
- [ ] Page loads quickly (< 3s)
- [ ] No console errors

### 11. Security Tests
- [ ] Check CSP in DevTools Network tab
- [ ] No sensitive data in console logs
- [ ] API keys not exposed in source

### 12. Email Tests (if configured)
- [ ] Submit Apply form
- [ ] Check if email would be sent (console logs)
- [ ] Verify Resend API key is working

## Test Results

Date: _______________
Tested by: _______________

### Issues Found:
1. 
2. 
3. 

### Notes: