import React from 'react';
import DocBreadcrumbs from '@theme-original/DocBreadcrumbs';
import type DocBreadcrumbsType from '@theme/DocBreadcrumbs';
import type { WrapperProps } from '@docusaurus/types';
import CopyPageButton from '@site/src/components/CopyPageButton';

type Props = WrapperProps<typeof DocBreadcrumbsType>;

export default function DocBreadcrumbsWrapper(props: Props): JSX.Element {
  return (
    <div className="breadcrumbs-row">
      <DocBreadcrumbs {...props} />
      <CopyPageButton />
    </div>
  );
}
