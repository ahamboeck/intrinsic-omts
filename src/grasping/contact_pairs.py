"""Pure mesh contact-pair search entry point (not implemented in step 1)."""

from src.grasping.types import (
  ContactPair,
  ContactPairOptions,
  ParallelJawLimits,
  TriangleMesh,
)


def propose_contact_pairs(
  mesh: TriangleMesh,
  gripper: ParallelJawLimits,
  *,
  options: ContactPairOptions,
) -> list[ContactPair]:
  """Reserve the search contract without pretending a search found no pairs."""
  raise NotImplementedError("Contact-pair search is not implemented yet.")
