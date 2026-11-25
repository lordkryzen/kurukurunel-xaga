// SPDX-License-Identifier: GPL-2.0
/* Native character-device shared-memory ABI, backed by the existing TEE pool.
 * Native 5.10 tee_mmap + SHM_ID/SHM_RELEASE are the user-visible contract.
 * A list-owner and each VMA hold references, so either release/unmap order is
 * safe. Existing dma-buf references also protect in-flight parameter users.
 */
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <tee_drv.h>
#include "tee_private.h"

struct tee_native_mapping {
	struct list_head link;
	struct kref refs;
	struct tee_context *ctx;
	struct mm_struct *mm;
	struct tee_shm *shm;
	unsigned long address;
};

static LIST_HEAD(native_mappings);
static DEFINE_MUTEX(native_mapping_lock);

static void native_mapping_free(struct kref *ref)
{
	struct tee_native_mapping *mapping = container_of(ref, struct tee_native_mapping, refs);

	isee_shm_put(mapping->shm);
	mmdrop(mapping->mm);
	kfree(mapping);
}

static void native_mapping_unlist(struct tee_native_mapping *mapping)
{
	struct tee_shm *shm = mapping->shm;

	list_del(&mapping->link);
	/* Retire the public ID now; active VMA/parameter references retain pages. */
	mutex_lock(&shm->teedev->mutex);
	if (shm->id >= 0) {
		idr_remove(&shm->teedev->idr, shm->id);
		shm->id = -1;
	}
	mutex_unlock(&shm->teedev->mutex);
	kref_put(&mapping->refs, native_mapping_free);
}

static void native_vma_open(struct vm_area_struct *vma)
{
	struct tee_native_mapping *mapping = vma->vm_private_data;

	kref_get(&mapping->refs);
}

static void native_vma_close(struct vm_area_struct *vma)
{
	struct tee_native_mapping *mapping = vma->vm_private_data;

	kref_put(&mapping->refs, native_mapping_free);
}

static const struct vm_operations_struct native_vm_ops = {
	.open = native_vma_open,
	.close = native_vma_close,
};

int tee_native_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct tee_context *ctx = file->private_data;
	struct tee_native_mapping *mapping;
	struct tee_shm *shm;
	size_t size;
	int ret;

	if (!ctx || vma->vm_end <= vma->vm_start || vma->vm_pgoff ||
	    !(vma->vm_flags & VM_SHARED) || (vma->vm_flags & VM_EXEC))
		return -EINVAL;
	size = vma->vm_end - vma->vm_start;
	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return -ENOMEM;
	shm = isee_shm_alloc(ctx, size, TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(shm)) {
		ret = PTR_ERR(shm);
		goto free_mapping;
	}
	ret = remap_pfn_range(vma, vma->vm_start, shm->paddr >> PAGE_SHIFT,
			      size, vma->vm_page_prot);
	if (ret) {
		isee_shm_put(shm);
		goto free_mapping;
	}
	mapping->ctx = ctx;
	mapping->mm = vma->vm_mm;
	mmgrab(mapping->mm);
	mapping->shm = shm;
	mapping->address = vma->vm_start;
	kref_init(&mapping->refs); /* Native handle/list ownership. */
	kref_get(&mapping->refs); /* Initial VMA ownership. */
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
	vm_flags_clear(vma, VM_MAYEXEC);
	vma->vm_ops = &native_vm_ops;
	vma->vm_private_data = mapping;
	mutex_lock(&native_mapping_lock);
	list_add_tail(&mapping->link, &native_mappings);
	mutex_unlock(&native_mapping_lock);
	return 0;
free_mapping:
	kfree(mapping);
	return ret;
}

int tee_native_shm_id(struct tee_context *ctx, unsigned long address)
{
	struct tee_native_mapping *mapping;
	int id = -EINVAL;

	mutex_lock(&native_mapping_lock);
	list_for_each_entry(mapping, &native_mappings, link) {
		if (mapping->ctx == ctx && mapping->mm == current->mm &&
		    mapping->address == address && kref_read(&mapping->refs) > 1) {
			id = mapping->shm->id;
			break;
		}
	}
	mutex_unlock(&native_mapping_lock);
	return id;
}

int tee_native_shm_release(struct tee_context *ctx, unsigned long id)
{
	struct tee_native_mapping *mapping;
	int ret = -EINVAL;

	if (!id || id > INT_MAX)
		return -EINVAL;
	mutex_lock(&native_mapping_lock);
	list_for_each_entry(mapping, &native_mappings, link) {
		if (mapping->ctx == ctx && (unsigned long)mapping->shm->id == id) {
			native_mapping_unlist(mapping);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&native_mapping_lock);
	return ret;
}

void tee_native_shm_close_context(struct tee_context *ctx)
{
	struct tee_native_mapping *mapping, *next;

	mutex_lock(&native_mapping_lock);
	list_for_each_entry_safe(mapping, next, &native_mappings, link) {
		if (mapping->ctx == ctx)
			native_mapping_unlist(mapping);
	}
	mutex_unlock(&native_mapping_lock);
}
