# Opoch Documentation Management Guide

How to add, edit, and organize documentation via GitHub - **no code editing required**.

---

## Quick Summary

| Action | What You Need to Do |
|--------|---------------------|
| **Edit existing doc** | Edit the `.md` file on GitHub |
| **Add new doc** | Create `.md` file in the right folder |
| **Create new category** | Create a new folder + `_category_.json` |
| **Change order** | Edit `sidebar_position` in frontmatter |

**Everything is automatic!** No TypeScript/code files to edit.

---

## Folder Structure

```
docs/
├── index.md                 ← Homepage (Overview)
├── verify.md                ← Standalone page
├── truth/                   ← "The Truth" category
│   ├── _category_.json      ← Category settings (name, order)
│   ├── introduction.md
│   ├── kernel-proof.md
│   └── ...
├── impact/                  ← "The Impact" category
│   ├── _category_.json
│   ├── physics.md
│   └── consciousness.md
└── technology/              ← "The Technology" category
    ├── _category_.json
    ├── the-validation.md
    └── the-frontier.md
```

---

## Adding a New Document

### Step 1: Create the file

Create a new `.md` file in the appropriate folder.

**Example:** Adding "Ethics" to The Impact

Create: `docs/impact/ethics.md`

### Step 2: Add frontmatter at the top

```markdown
---
sidebar_position: 3
title: Ethics
description: Deriving ethics from the kernel
---

# Ethics

Your content here...
```

**That's it!** The doc will appear automatically in the sidebar.

---

## Frontmatter Reference

Every doc needs this at the top between `---` markers:

```markdown
---
sidebar_position: 1
title: Your Title
description: One-line description
---
```

| Field | What it does |
|-------|--------------|
| `sidebar_position` | Order in sidebar (1, 2, 3...) |
| `title` | Page title and sidebar label |
| `description` | SEO description |

---

## Creating a New Category

### Step 1: Create a folder

```
docs/applications/
```

### Step 2: Add `_category_.json` in that folder

Create: `docs/applications/_category_.json`

```json
{
  "label": "Applications",
  "position": 5,
  "collapsed": true,
  "collapsible": true
}
```

| Field | What it does |
|-------|--------------|
| `label` | Category name in sidebar |
| `position` | Order among other categories |
| `collapsed` | Start collapsed? `true` or `false` |
| `collapsible` | Can users collapse it? `true` or `false` |

### Step 3: Add your docs

Create markdown files in that folder:
```
docs/applications/ai-reasoning.md
docs/applications/discovery.md
```

**Done!** The category appears automatically.

---

## Creating Sub-Categories

Just create a folder inside a folder:

```
docs/truth/proofs/
docs/truth/proofs/_category_.json
docs/truth/proofs/kernel.md
docs/truth/proofs/reasoning.md
```

---

## Controlling Order

### Document order within a category

Edit `sidebar_position` in the document's frontmatter:

```markdown
---
sidebar_position: 1   ← First
title: Introduction
---
```

```markdown
---
sidebar_position: 2   ← Second
title: Kernel Proof
---
```

### Category order

Edit `position` in `_category_.json`:

```json
{
  "label": "The Truth",
  "position": 2
}
```

---

## Adding Images

1. Upload image to `static/` folder
2. Reference in markdown:

```markdown
![Description](/my-image.png)
```

---

## Adding PDFs

1. Upload PDF to `static/` folder
2. Link in markdown:

```markdown
[Download PDF](/my-paper.pdf)
```

---

## Linking Between Docs

```markdown
See [The Engine](/truth/the-engine) for details.

See the [introduction section](/truth/introduction#section-name).
```

---

## Common Markdown Patterns

### Info boxes

```markdown
:::info Note
Important information here.
:::

:::warning
Warning message here.
:::

:::tip
Helpful tip here.
:::
```

### Math equations (IMPORTANT!)

**Always wrap LaTeX in dollar signs:**

```markdown
Inline math: $E = mc^2$

Block math (on its own line):
$$
\Pi := \lim_{c \to \infty} \Pi_c
$$

Boxed equations:
$$\boxed{x = y}$$
```

**Common mistakes that break the build:**

| Wrong | Correct |
|-------|---------|
| `\boxed{x=y}` | `$$\boxed{x=y}$$` |
| `D^\*` in text | `$D^*$` |
| `{-1, 0, +1}` | `\{-1, 0, +1\}` |
| `⸻` (unicode dash) | `---` (three hyphens) |

### Code blocks

````markdown
```python
def hello():
    print("Hello")
```
````

---

## File Naming Rules

- Use lowercase: `my-document.md` ✓
- Use hyphens for spaces: `kernel-proof.md` ✓
- No spaces: `kernel proof.md` ✗
- No special characters

---

## Checklist

- [ ] Created `.md` file in correct folder
- [ ] Added frontmatter (title, description, sidebar_position)
- [ ] Committed to GitHub
- [ ] Wait ~2 min for auto-deploy

---

## Example: Adding a Complete New Section

Let's add "Research" as a new top-level category with two docs.

**1. Create folder:** `docs/research/`

**2. Create category config:** `docs/research/_category_.json`
```json
{
  "label": "Research",
  "position": 5,
  "collapsed": true,
  "collapsible": true
}
```

**3. Create first doc:** `docs/research/papers.md`
```markdown
---
sidebar_position: 1
title: Published Papers
description: Academic papers on the theory
---

# Published Papers

Content here...
```

**4. Create second doc:** `docs/research/collaborations.md`
```markdown
---
sidebar_position: 2
title: Collaborations
description: Research collaborations
---

# Collaborations

Content here...
```

**5. Commit to GitHub**

**Done!** "Research" category appears automatically with both docs.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Doc not appearing | Check frontmatter has `---` at start and end |
| Wrong order | Check `sidebar_position` numbers |
| Category not showing | Make sure `_category_.json` is valid JSON |
| Build failed | See "Build Errors" below |

### Build Errors

**"Could not parse expression with acorn"** - Usually means:
- LaTeX without `$...$` wrapping
- Curly braces `{}` not escaped as `\{\}`
- Unicode special characters (use plain ASCII)

**"VFileMessage" error** - Same as above, MDX parsing failed

**How to debug:**
1. Look at the error line number
2. Check for unescaped `{`, `}`, `<`, `>`
3. Wrap all math in `$...$` or `$$...$$`
4. Use `---` not unicode dashes

---

## Questions?

Look at existing files for examples - they all follow this pattern.
